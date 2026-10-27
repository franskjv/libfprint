// Goodix Tls driver for libfprint

// Copyright (C) 2021 Alexander Meiler <alex.meiler@protonmail.com>
// Copyright (C) 2021 Matthieu CHARETTE <matthieu.charette@gmail.com>
// Copyright (C) 2021 Michael Teuscher <michael.teuscher@pm.me>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

#include "fp-device.h"
#include "fp-image-device.h"
#include "fp-image.h"
#include "fpi-assembling.h"
#include "fpi-context.h"
#include "fpi-image-device.h"
#include "fpi-image.h"
#include "fpi-ssm.h"
#include "glibconfig.h"
#include "gusb/gusb-device.h"
#include <stdio.h>
#include <stdlib.h>
#define FP_COMPONENT "goodixtls53xd"

#include <glib.h>
#include <string.h>

#include "drivers_api.h"
#include "goodix.h"
#include "goodix_proto.h"
#include "goodix53xd.h"

#include <math.h>

#define GOODIX53XD_WIDTH 64
#define GOODIX53XD_HEIGHT 80
#define GOODIX53XD_SCAN_WIDTH 64
#define GOODIX53XD_FRAME_SIZE (GOODIX53XD_WIDTH * GOODIX53XD_HEIGHT)
// For every 4 pixels there are 6 bytes and there are 8 extra start bytes and 5
// extra end
#define GOODIX53XD_RAW_FRAME_SIZE                                               \
    ((GOODIX53XD_HEIGHT * GOODIX53XD_SCAN_WIDTH) / 4 * 6)
#define GOODIX53XD_CAP_FRAMES 3 // Number of frames averaged per press
// Offset added after background subtraction so pixels stay unsigned: the
// background extrapolated at the finger DAC goes above 4095
#define GOODIX53XD_BG_OFFSET 0x2000
// With the OTP DAC values, the finger-free sensor saturates at 4095. Each DAC
// step lowers every pixel by ~73, linearly. Calibration frames are taken at
// two higher DAC offsets where nothing saturates, and the background at the
// finger DAC is extrapolated from them. Finger detection polls at the first
// calibration offset.
#define GOODIX53XD_DETECT_DAC_STEP 20
#define GOODIX53XD_CALIB_DAC_STEP 40
// Finger-free checks of the calibration frames: median of the detection
// frame above this level, and few zeros in the calibration frame
#define GOODIX53XD_EMPTY_MIN_LEVEL 2500
#define GOODIX53XD_EMPTY_MAX_ZERO_PERCENT 5
// Finger detection polls images: the FDT thresholds are sensor specific and
// the hardcoded ones trigger without finger on other sensors
#define GOODIX53XD_POLL_DELAY 100 // ms between two polled frames
#define GOODIX53XD_FINGER_NOISE_FACTOR 4
#define GOODIX53XD_FINGER_MIN_DELTA 16 // 12 bit units
#define GOODIX53XD_MAX_BACKGROUND_NOISE 100 // ~7 without finger
// A finger still landing gives a partial, blurry image: the capture waits
// for the delta to the background to stop changing, then rejects presses
// that are too light
#define GOODIX53XD_SETTLE_PERCENT 5 // max delta change between two polls
#define GOODIX53XD_SETTLE_MAX_POLLS 8
#define GOODIX53XD_MIN_CONTACT_DELTA 1900 // $GOODIX53XD_MIN_CONTACT overrides
// Each image only covers a small part of the finger
#define GOODIX53XD_ENROLL_STAGES 20
#define GOODIX53XD_TEMPLATE_MAX 50
// Verify scans with fewer SIFT keypoints, or a stronger signal, are asked
// again: offline they caught half of the failed matches and almost no
// matching scan. $GOODIX53XD_MAX_AMPLITUDE overrides the amplitude.
#define GOODIX53XD_MIN_KEYPOINTS 33
#define GOODIX53XD_MAX_AMPLITUDE 230
#define GOODIX53XD_ACTIVATE_ATTEMPTS 3
#define GOODIX53XD_ACTIVATE_RETRY_DELAY 200 // ms
// Stale replies dropped when opening, see drain_input()
#define GOODIX53XD_DRAIN_MAX_READS 64
#define GOODIX53XD_DRAIN_SIZE 0x10000
#define GOODIX53XD_DRAIN_TIMEOUT 100 // ms
#define GOODIX53XD_STRETCH_PERCENTILE 2
// The outermost pixels have extreme values, they are cropped from the image
#define GOODIX53XD_BORDER 2
#define GOODIX53XD_IMG_WIDTH (GOODIX53XD_WIDTH - 2 * GOODIX53XD_BORDER)
#define GOODIX53XD_IMG_HEIGHT (GOODIX53XD_HEIGHT - 2 * GOODIX53XD_BORDER)
#define GOODIX53XD_IMG_SIZE (GOODIX53XD_IMG_WIDTH * GOODIX53XD_IMG_HEIGHT)

typedef unsigned short Goodix53xdPix;

typedef enum {
    GOODIX53XD_CAPTURE_BACKGROUND,
    GOODIX53XD_CAPTURE_WAIT_FINGER_ON,
    GOODIX53XD_CAPTURE_SETTLE,
    GOODIX53XD_CAPTURE_PRESS,
    GOODIX53XD_CAPTURE_WAIT_FINGER_OFF,
} Goodix53xdCaptureMode;

struct _FpiDeviceGoodixTls53XD {
  FpiDeviceGoodixTls parent;

  guint8* otp;

  GSList* frames;

  // Background at the finger DAC, extrapolated from the calibration frames
  Goodix53xdPix empty_img[GOODIX53XD_FRAME_SIZE];
  // Finger-free frames at the detection and calibration DAC offsets
  Goodix53xdPix detect_img[GOODIX53XD_FRAME_SIZE];
  Goodix53xdPix calib_img[GOODIX53XD_FRAME_SIZE];

  // DAC offset of the finger frames ($GOODIX53XD_DAC_OFFSET), and of the
  // next frame read
  int finger_dac_offset;
  int dac_offset;

  Goodix53xdCaptureMode capture_mode;
  guint background_frames;
  guint finger_threshold;
  guint min_contact;
  guint max_amplitude;
  guint last_amplitude;
  guint last_delta;
  guint settle_polls;

  // Debug dumps, see dump_path()
  gchar* dump_session;
  guint dump_press;

  guint activate_attempts;

  FpiSsm* scan_ssm;
  gboolean polling;
  gboolean deactivating;
};

G_DECLARE_FINAL_TYPE(FpiDeviceGoodixTls53XD, fpi_device_goodixtls53xd, FPI,
                     DEVICE_GOODIXTLS53XD, FpiDeviceGoodixTls);

G_DEFINE_TYPE(FpiDeviceGoodixTls53XD, fpi_device_goodixtls53xd,
              FPI_TYPE_DEVICE_GOODIXTLS);

// ---- ACTIVE SECTION START ----

enum activate_states {
    ACTIVATE_READ_AND_NOP,
    ACTIVATE_ENABLE_CHIP,
    ACTIVATE_NOP,
    ACTIVATE_CHECK_FW_VER,
    ACTIVATE_CHECK_PSK,
    ACTIVATE_RESET,
    ACTIVATE_OTP,
    ACTIVATE_SET_MCU_IDLE,
    ACTIVATE_SET_MCU_CONFIG,
    ACTIVATE_NUM_STATES,
};

static void check_none(FpDevice *dev, gpointer user_data, GError *error) {
  if (error) {
    fpi_ssm_mark_failed(user_data, error);
    return;
  }

  fpi_ssm_next_state(user_data);
}

static void check_firmware_version(FpDevice *dev, gchar *firmware,
                                   gpointer user_data, GError *error) {
  if (error) {
    fpi_ssm_mark_failed(user_data, error);
    return;
  }

  fp_dbg("Device firmware: \"%s\"", firmware);

  if (strcmp(firmware, GOODIX_53XD_FIRMWARE_VERSION)) {
    g_set_error(&error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                "Invalid device firmware: \"%s\"", firmware);
    fpi_ssm_mark_failed(user_data, error);
    return;
  }

  fpi_ssm_next_state(user_data);
}

static void check_reset(FpDevice *dev, gboolean success, guint16 number,
                        gpointer user_data, GError *error) {
  if (error) {
    fpi_ssm_mark_failed(user_data, error);
    return;
  }

  if (!success) {
    g_set_error(&error, G_IO_ERROR, G_IO_ERROR_FAILED,
                "Failed to reset device");
    fpi_ssm_mark_failed(user_data, error);
    return;
  }

  fp_dbg("Device reset number: %d", number);

  if (number != GOODIX_53XD_RESET_NUMBER) {
    g_set_error(&error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                "Invalid device reset number: %d", number);
    fpi_ssm_mark_failed(user_data, error);
    return;
  }

  fpi_ssm_next_state(user_data);
}

static void check_preset_psk_read(FpDevice *dev, gboolean success,
                                  guint32 flags, guint8 *psk, guint16 length,
                                  gpointer user_data, GError *error) {
  g_autofree gchar *psk_str = data_to_str(psk, length);

  if (error) {
    fpi_ssm_mark_failed(user_data, error);
    return;
  }

  if (!success) {
    g_set_error(&error, G_IO_ERROR, G_IO_ERROR_FAILED,
                "Failed to read PSK from device");
    fpi_ssm_mark_failed(user_data, error);
    return;
  }

  fp_dbg("Device PSK: 0x%s", psk_str);
  fp_dbg("Device PSK flags: 0x%08x", flags);

  if (flags != GOODIX_53XD_PSK_FLAGS) {
    g_set_error(&error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                "Invalid device PSK flags: 0x%08x", flags);
    fpi_ssm_mark_failed(user_data, error);
    return;
  }

  if (length != sizeof(goodix_53xd_psk_0)) {
    g_set_error(&error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                "Invalid device PSK: 0x%s", psk_str);
    fpi_ssm_mark_failed(user_data, error);
    return;
  }

  if (memcmp(psk, goodix_53xd_psk_0, sizeof(goodix_53xd_psk_0))) {
    g_set_error(&error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                "Invalid device PSK: 0x%s", psk_str);
    fpi_ssm_mark_failed(user_data, error);
    return;
  }

  fpi_ssm_next_state(user_data);
}
static void check_idle(FpDevice* dev, gpointer user_data, GError* err)
{

    if (err) {
        fpi_ssm_mark_failed(user_data, err);
        return;
    }
    fpi_ssm_next_state(user_data);
}
static void check_config_upload(FpDevice* dev, gboolean success,
                                gpointer user_data, GError* error)
{
    if (error) {
        fpi_ssm_mark_failed(user_data, error);
    }
    else if (!success) {
        fpi_ssm_mark_failed(user_data,
                            g_error_new(FP_DEVICE_ERROR, FP_DEVICE_ERROR_PROTO,
                                        "failed to upload mcu config"));
    }
    else {
        fpi_ssm_next_state(user_data);
    }
}
static void check_powerdown_scan_freq(FpDevice* dev, gboolean success,
                                      gpointer user_data, GError* error)
{
    if (error) {
        fpi_ssm_mark_failed(user_data, error);
    }
    else if (!success) {
        fpi_ssm_mark_failed(user_data,
                            g_error_new(FP_DEVICE_ERROR, FP_DEVICE_ERROR_PROTO,
                                        "failed to set powerdown freq"));
    }
    fpi_ssm_next_state(user_data);
}

enum otp_write_states {
    OTP_WRITE_1,
    OTP_WRITE_2,

    OTP_WRITE_NUM,
};

static void otp_write_run(FpiSsm* ssm, FpDevice* dev)
{
    /*FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);
    switch (fpi_ssm_get_cur_state(ssm)) {
    case OTP_WRITE_1:
        guint8 sensor1[] = {0x0a, 0x02};
        goodix_send_write_sensor_register(
        dev, 0x022c, sensor1, check_none,
        ssm);
        break;
    case OTP_WRITE_2:
        guint8 sensor2[] = {0x0a, 0x03};
        goodix_send_write_sensor_register(
            dev, 0x022c, sensor2, check_none,
            ssm);
        fpi_ssm_next_state(ssm);
        break;
    }*/
}

static void read_otp_callback(FpDevice* dev, guint8* data, guint16 len,
                              gpointer ssm, GError* err)
{
    if (err) {
        fpi_ssm_mark_failed(ssm, err);
        return;
    }
    if (len < 64) {
        fpi_ssm_mark_failed(ssm, g_error_new(FP_DEVICE_ERROR,
                                             FP_DEVICE_ERROR_DATA_INVALID,
                                             "OTP is invalid (len: %d)", 64));
        return;
    }
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);
    self->otp = malloc(64);
    memcpy(self->otp, data, len);
    fpi_ssm_next_state(ssm);
}

static void activate_run_state(FpiSsm* ssm, FpDevice* dev)
{

    switch (fpi_ssm_get_cur_state(ssm)) {
    case ACTIVATE_READ_AND_NOP:
        // Nop seems to clear the previous command buffer. But we are
        // unable to do so.
        goodix_start_read_loop(dev);
        goodix_send_nop(dev, check_none, ssm);
        break;

    case ACTIVATE_ENABLE_CHIP:
      goodix_send_enable_chip(dev, TRUE, check_none, ssm);
      break;

    case ACTIVATE_NOP:
      goodix_send_nop(dev, check_none, ssm);
      break;

    case ACTIVATE_CHECK_FW_VER:
      goodix_send_firmware_version(dev, check_firmware_version, ssm);
      break;

    case ACTIVATE_CHECK_PSK:
      goodix_send_preset_psk_read(dev, GOODIX_53XD_PSK_FLAGS, 32,
                                  check_preset_psk_read, ssm);
      break;

    case ACTIVATE_RESET:
      goodix_send_reset(dev, TRUE, 20, check_reset, ssm);
      break;

    case ACTIVATE_OTP:
      goodix_send_read_otp(dev, read_otp_callback, ssm);
      break;

    case ACTIVATE_SET_MCU_IDLE:
        goodix_send_mcu_switch_to_idle_mode(dev, 20, check_idle, ssm);
        break;

    case ACTIVATE_SET_MCU_CONFIG:
        goodix_send_upload_config_mcu(dev, goodix_53xd_config,
                                      sizeof(goodix_53xd_config), NULL,
                                      check_config_upload, ssm);
        break;
    }
}

static void tls_activation_complete(FpDevice* dev, gpointer user_data,
                                    GError* error)
{
    FpImageDevice* image_dev = FP_IMAGE_DEVICE(dev);

    if (error) {
        fp_err("failed to complete tls activation: %s", error->message);
    }
    // Report the error too, otherwise the activation never completes
    fpi_image_device_activate_complete(image_dev, error);
}

static void activate_complete(FpiSsm* ssm, FpDevice* dev, GError* error);

static void activate_start(FpDevice* dev, gpointer user_data)
{
    fpi_ssm_start(fpi_ssm_new(dev, activate_run_state, ACTIVATE_NUM_STATES),
                  activate_complete);
}

static void activate_complete(FpiSsm* ssm, FpDevice* dev, GError* error)
{
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);

    G_DEBUG_HERE();
    if (!error) {
        goodix_tls(dev, tls_activation_complete, NULL);
        return;
    }

    // The reader is runtime suspended after 2 s idle (udev autosuspend
    // hwdb), and the first command after resuming may time out
    if (g_error_matches(error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT) &&
        ++self->activate_attempts < GOODIX53XD_ACTIVATE_ATTEMPTS) {
        fp_dbg("Activation timed out (%s), retrying", error->message);
        g_error_free(error);
        goodix_reset_state(dev);
        fpi_device_add_timeout(dev, GOODIX53XD_ACTIVATE_RETRY_DELAY,
                               activate_start, NULL, NULL);
        return;
    }

    fp_err("failed during activation: %s (code: %d)", error->message,
           error->code);
    fpi_image_device_activate_complete(FP_IMAGE_DEVICE(dev), error);
}

// ---- ACTIVE SECTION END ----

// -----------------------------------------------------------------------------

// ---- SCAN SECTION START ----

enum SCAN_STAGES {
    SCAN_STAGE_CALIBRATE,
    SCAN_STAGE_WAIT,
    SCAN_STAGE_SWITCH_TO_FDT_MODE,
    SCAN_STAGE_GET_IMG,

    SCAN_STAGE_NUM,
};

static void check_none_cmd(FpDevice* dev, guint8* data, guint16 len,
                           gpointer ssm, GError* err)
{
    if (err) {
        fpi_ssm_mark_failed(ssm, err);
        return;
    }
    fpi_ssm_next_state(ssm);
}

// Bitdepth is 12, but we have to fit it in a byte
static unsigned char squash(int v) { return v / 16; }

static void decode_frame(Goodix53xdPix frame[GOODIX53XD_FRAME_SIZE],
                         const guint8* raw_frame)
{
    Goodix53xdPix uncropped[GOODIX53XD_SCAN_WIDTH * GOODIX53XD_HEIGHT];
    Goodix53xdPix* pix = uncropped;
    for (int i = 0; i < GOODIX53XD_RAW_FRAME_SIZE; i += 6) {
        const guint8* chunk = raw_frame + i;
        *pix++ = ((chunk[0] & 0xf) << 8) + chunk[1];
        *pix++ = (chunk[3] << 4) + (chunk[0] >> 4);
        *pix++ = ((chunk[5] & 0xf) << 8) + chunk[2];
        *pix++ = (chunk[4] << 4) + (chunk[5] >> 4);
    }

    for (int y = 0; y != GOODIX53XD_HEIGHT; ++y) {
        for (int x = 0; x != GOODIX53XD_WIDTH; ++x) {
            const int idx = x + y * GOODIX53XD_SCAN_WIDTH;
            frame[x + y * GOODIX53XD_WIDTH] = uncropped[idx];
        }
    }
}
static int goodix_cmp_short(const void* a, const void* b)
{
    return (int) (*(short*) a - *(short*) b);
}

static void rotate_frame(Goodix53xdPix frame[GOODIX53XD_FRAME_SIZE])
{
    Goodix53xdPix buff[GOODIX53XD_FRAME_SIZE];

    for (int y = 0; y != GOODIX53XD_HEIGHT; ++y) {
        for (int x = 0; x != GOODIX53XD_WIDTH; ++x) {
            buff[x * GOODIX53XD_WIDTH + y] = frame[x + y * GOODIX53XD_WIDTH];
        }
    }
    memcpy(frame, buff, GOODIX53XD_FRAME_SIZE);
}
static void squash_frame(Goodix53xdPix* frame, guint8* squashed)
{
    for (int i = 0; i != GOODIX53XD_FRAME_SIZE; ++i) {
        squashed[i] = squash(frame[i]);
    }
}
static int goodix_cmp_int(const void* a, const void* b)
{
    return *(const int*) a - *(const int*) b;
}

/**
 * @brief Squashes the 12 bit pixels of a frame into the 8 bit pixels used
 * by libfprint, cropping GOODIX53XD_BORDER pixels on each side.
 * @details Each row gets its mean removed, the sensor has a row dependent
 * offset that shows as horizontal bands. The result is stretched between the
 * GOODIX53XD_STRETCH_PERCENTILE low and high percentiles, so that a few
 * extreme pixels don't flatten the ridges.
 *
 * @param frame
 * @param squashed GOODIX53XD_IMG_SIZE pixels
 */
static void squash_frame_rows(Goodix53xdPix* frame, guint8* squashed)
{
    int values[GOODIX53XD_IMG_SIZE];
    int sorted[GOODIX53XD_IMG_SIZE];

    for (int y = 0; y != GOODIX53XD_IMG_HEIGHT; ++y) {
        const Goodix53xdPix* row = frame +
                                   (y + GOODIX53XD_BORDER) * GOODIX53XD_WIDTH +
                                   GOODIX53XD_BORDER;
        int sum = 0;

        for (int x = 0; x != GOODIX53XD_IMG_WIDTH; ++x) {
            sum += row[x];
        }
        for (int x = 0; x != GOODIX53XD_IMG_WIDTH; ++x) {
            values[x + y * GOODIX53XD_IMG_WIDTH] =
                row[x] - sum / GOODIX53XD_IMG_WIDTH;
        }
    }

    memcpy(sorted, values, sizeof(values));
    qsort(sorted, GOODIX53XD_IMG_SIZE, sizeof(int), goodix_cmp_int);
    const int low =
        sorted[GOODIX53XD_IMG_SIZE * GOODIX53XD_STRETCH_PERCENTILE / 100];
    const int high = sorted[GOODIX53XD_IMG_SIZE *
                            (100 - GOODIX53XD_STRETCH_PERCENTILE) / 100];

    for (int i = 0; i != GOODIX53XD_IMG_SIZE; ++i) {
        if (high == low) {
            squashed[i] = 0;
            continue;
        }
        squashed[i] = CLAMP((values[i] - low) * 0xff / (high - low), 0, 0xff);
    }
}

/**
 * @brief Subtracts the background from the frame
 * @details The difference is shifted by GOODIX53XD_BG_OFFSET so it stays
 * unsigned, squash_frame_rows() then stretches it to 8 bits.
 *
 * @param frame
 * @param background
 */
static void postprocess_frame(Goodix53xdPix frame[GOODIX53XD_FRAME_SIZE],
                              Goodix53xdPix background[GOODIX53XD_FRAME_SIZE])
{
    for (int i = 0; i != GOODIX53XD_FRAME_SIZE; ++i) {
        frame[i] = frame[i] + GOODIX53XD_BG_OFFSET - background[i];
    }
}

static gboolean in_image(int i)
{
    const int x = i % GOODIX53XD_WIDTH;
    const int y = i / GOODIX53XD_WIDTH;

    return x >= GOODIX53XD_BORDER && x < GOODIX53XD_WIDTH - GOODIX53XD_BORDER &&
           y >= GOODIX53XD_BORDER && y < GOODIX53XD_HEIGHT - GOODIX53XD_BORDER;
}

/**
 * @brief Standard deviation of the background subtracted signal, in the
 * cropped image
 *
 * @param frame postprocessed frame
 */
static guint signal_amplitude(const Goodix53xdPix frame[GOODIX53XD_FRAME_SIZE])
{
    double sum = 0;
    double sum_sq = 0;
    guint count = 0;

    for (int i = 0; i != GOODIX53XD_FRAME_SIZE; ++i) {
        if (!in_image(i)) {
            continue;
        }
        sum += frame[i];
        sum_sq += (double) frame[i] * frame[i];
        count++;
    }

    const double mean = sum / count;
    return (guint) sqrt(MAX(sum_sq / count - mean * mean, 0));
}

/**
 * @brief Averages the captured frames of a single press into one frame
 *
 * @param frames
 * @param averaged
 */
static void average_frames(GSList* frames,
                           Goodix53xdPix averaged[GOODIX53XD_FRAME_SIZE])
{
    guint32 sum[GOODIX53XD_FRAME_SIZE] = {0};
    const guint count = g_slist_length(frames);

    for (GSList* l = frames; l != NULL; l = l->next) {
        const Goodix53xdPix* frame = l->data;
        for (int i = 0; i != GOODIX53XD_FRAME_SIZE; ++i) {
            sum[i] += frame[i];
        }
    }

    for (int i = 0; i != GOODIX53XD_FRAME_SIZE; ++i) {
        averaged[i] = sum[i] / count;
    }
}

/**
 * @brief Mean absolute difference between two frames
 *
 * @param a
 * @param b
 */
static guint frame_delta(const Goodix53xdPix a[GOODIX53XD_FRAME_SIZE],
                         const Goodix53xdPix b[GOODIX53XD_FRAME_SIZE])
{
    guint64 sum = 0;

    for (int i = 0; i != GOODIX53XD_FRAME_SIZE; ++i) {
        sum += ABS((int) a[i] - (int) b[i]);
    }

    return sum / GOODIX53XD_FRAME_SIZE;
}

/**
 * @brief Path of a debug dump file of the current press, NULL when
 * $GOODIX53XD_DUMP_DIR is unset
 * @details Files are named <session>-<press>-<suffix>, the session being the
 * time the first file was dumped, so successive runs don't overwrite.
 *
 * @param self
 * @param suffix
 */
static gchar* dump_path(FpiDeviceGoodixTls53XD* self, const gchar* suffix)
{
    const gchar* dir = g_getenv("GOODIX53XD_DUMP_DIR");

    if (dir == NULL) {
        return NULL;
    }

    if (self->dump_session == NULL) {
        g_autoptr(GDateTime) now = g_date_time_new_now_local();
        self->dump_session = g_date_time_format(now, "%Y%m%d-%H%M%S");
    }

    return g_strdup_printf("%s/%s-%03u-%s", dir, self->dump_session,
                           self->dump_press, suffix);
}

static void dump_file(const gchar* path, const gchar* header,
                      const guint8* data, gsize data_len)
{
    const gsize header_len = strlen(header);
    g_autofree guint8* contents = g_malloc(header_len + data_len);

    memcpy(contents, header, header_len);
    if (data_len > 0) {
        memcpy(contents + header_len, data, data_len);
    }

    g_autoptr(GError) error = NULL;
    if (!g_file_set_contents(path, (gchar*) contents, header_len + data_len,
                             &error)) {
        fp_warn("Failed to dump %s: %s", path, error->message);
        return;
    }
    fp_dbg("Dumped %s", path);
}

/**
 * @brief Saves a raw frame as a 16 bit PGM, for debugging only
 *
 * @param self
 * @param suffix
 * @param frame
 */
static void dump_raw_frame(FpiDeviceGoodixTls53XD* self, const gchar* suffix,
                           const Goodix53xdPix frame[GOODIX53XD_FRAME_SIZE])
{
    g_autofree gchar* path = dump_path(self, suffix);

    if (path == NULL) {
        return;
    }

    // PGM stores 16 bit samples big endian
    guint8 data[GOODIX53XD_FRAME_SIZE * 2];
    for (int i = 0; i != GOODIX53XD_FRAME_SIZE; ++i) {
        data[2 * i] = frame[i] >> 8;
        data[2 * i + 1] = frame[i] & 0xff;
    }

    g_autofree gchar* header = g_strdup_printf(
        "P5\n%u %u\n65535\n", GOODIX53XD_WIDTH, GOODIX53XD_HEIGHT);
    dump_file(path, header, data, sizeof(data));
}

/**
 * @brief Saves everything about a press in $GOODIX53XD_DUMP_DIR, for
 * debugging only: background, raw frames, final image and contact delta
 *
 * @param self
 * @param img
 */
static void dump_press(FpiDeviceGoodixTls53XD* self, FpImage* img)
{
    g_autofree gchar* img_path = dump_path(self, "image.pgm");

    if (img_path == NULL) {
        return;
    }

    dump_raw_frame(self, "background.pgm", self->empty_img);
    dump_raw_frame(self, "detect.pgm", self->detect_img);
    dump_raw_frame(self, "calibration.pgm", self->calib_img);

    guint index = 0;
    for (GSList* l = self->frames; l != NULL; l = l->next) {
        g_autofree gchar* suffix = g_strdup_printf("frame%u.pgm", index++);
        dump_raw_frame(self, suffix, l->data);
    }

    g_autofree gchar* header =
        g_strdup_printf("P5\n%u %u\n255\n", img->width, img->height);
    dump_file(img_path, header, img->data, img->width * img->height);

    g_autofree gchar* info_path = dump_path(self, "info.txt");
    g_autofree gchar* info = g_strdup_printf(
        "delta=%u\nsettle_polls=%u\nfinger_threshold=%u\namplitude=%u\n",
        self->last_delta, self->settle_polls, self->finger_threshold,
        self->last_amplitude);
    dump_file(info_path, info, NULL, 0);

    self->dump_press++;
}

/**
 * @brief Background at the finger DAC from the two calibration frames
 * @details The DAC shifts every pixel linearly, so the value at the finger
 * DAC is extrapolated from the detection and calibration offsets.
 *
 * @param self
 */
static void extrapolate_background(FpiDeviceGoodixTls53XD* self)
{
    const int span = GOODIX53XD_CALIB_DAC_STEP - GOODIX53XD_DETECT_DAC_STEP;
    guint saturated = 0;

    for (int i = 0; i != GOODIX53XD_FRAME_SIZE; ++i) {
        const int detect = self->detect_img[i];
        const int calib = self->calib_img[i];
        const int value =
            detect + (detect - calib) * GOODIX53XD_DETECT_DAC_STEP / span;

        // A finger on the sensor is rejected by calibration_is_empty(), this
        // only catches a DAC offset leaving pixels saturated
        if (in_image(i) && detect >= 0xfff) {
            saturated++;
        }
        self->empty_img[i] = CLAMP(value, 0, 0xffff);
    }

    if (saturated > GOODIX53XD_IMG_SIZE / 100) {
        fp_warn("%u calibration pixels saturated, background is wrong",
                saturated);
    }
}

/**
 * @brief Whether the calibration frames were taken without finger
 * @details Without finger, the detection frame median is ~3770 and no pixel
 * of the calibration frame is 0. With a finger, the median is below ~920 and
 * almost every calibration pixel is 0.
 *
 * @param self
 */
static gboolean calibration_is_empty(FpiDeviceGoodixTls53XD* self)
{
    guint above = 0;
    guint zeros = 0;

    for (int i = 0; i != GOODIX53XD_FRAME_SIZE; ++i) {
        if (!in_image(i)) {
            continue;
        }
        above += self->detect_img[i] >= GOODIX53XD_EMPTY_MIN_LEVEL;
        zeros += self->calib_img[i] == 0;
    }

    fp_dbg("Calibration: %u%% above %d, %u%% at 0",
           above * 100 / GOODIX53XD_IMG_SIZE, GOODIX53XD_EMPTY_MIN_LEVEL,
           zeros * 100 / GOODIX53XD_IMG_SIZE);
    return above * 2 > GOODIX53XD_IMG_SIZE &&
           zeros * 100 < GOODIX53XD_IMG_SIZE * GOODIX53XD_EMPTY_MAX_ZERO_PERCENT;
}

static void save_frame(FpiDeviceGoodixTls53XD* self, guint8* raw)
{
    Goodix53xdPix* frame = malloc(GOODIX53XD_FRAME_SIZE * sizeof(Goodix53xdPix));
    decode_frame(frame, raw);
    self->frames = g_slist_append(self->frames, frame);
}

static void scan_on_read_img(FpDevice* dev, guint8* data, guint16 len,
                             gpointer ssm, GError* err)
{
    if (err) {
        fpi_ssm_mark_failed(ssm, err);
        return;
    }


    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);
    FpImageDevice* img_dev = FP_IMAGE_DEVICE(dev);
    Goodix53xdPix frame[GOODIX53XD_FRAME_SIZE];
    guint delta;

    switch (self->capture_mode) {
    case GOODIX53XD_CAPTURE_BACKGROUND:
        // Three frames without finger: at the detection offset, at the
        // calibration offset, then at the detection offset again, their
        // difference with the first one being the sensor noise
        if (self->background_frames == 0) {
            decode_frame(self->detect_img, data);
            dump_raw_frame(self, "empty-detect.pgm", self->detect_img);
            self->background_frames++;
            self->dac_offset =
                self->finger_dac_offset + GOODIX53XD_CALIB_DAC_STEP;
            fpi_ssm_jump_to_state(ssm, SCAN_STAGE_SWITCH_TO_FDT_MODE);
            return;
        }

        if (self->background_frames == 1) {
            decode_frame(self->calib_img, data);

            // A finger still on the sensor would become the background:
            // lifting it would then look like a press
            if (!calibration_is_empty(self)) {
                fp_dbg("Finger on the sensor during calibration, retrying");
                self->background_frames = 0;
                self->dac_offset =
                    self->finger_dac_offset + GOODIX53XD_DETECT_DAC_STEP;
                fpi_ssm_jump_to_state(ssm, SCAN_STAGE_WAIT);
                return;
            }

            self->background_frames++;
            extrapolate_background(self);
            self->dac_offset =
                self->finger_dac_offset + GOODIX53XD_DETECT_DAC_STEP;
            fpi_ssm_jump_to_state(ssm, SCAN_STAGE_SWITCH_TO_FDT_MODE);
            return;
        }

        decode_frame(frame, data);
        delta = frame_delta(frame, self->detect_img);

        // A finger landing during calibration shows up as noise: the
        // threshold derived from it would never be reached by a press
        if (delta > GOODIX53XD_MAX_BACKGROUND_NOISE) {
            fp_dbg("Background noise %u too high, calibrating again", delta);
            self->background_frames = 0;
            fpi_ssm_jump_to_state(ssm, SCAN_STAGE_WAIT);
            return;
        }

        self->finger_threshold = MAX(delta * GOODIX53XD_FINGER_NOISE_FACTOR,
                                     delta + GOODIX53XD_FINGER_MIN_DELTA);
        fp_dbg("Background noise: %u, finger threshold: %u", delta,
               self->finger_threshold);

        self->capture_mode = GOODIX53XD_CAPTURE_WAIT_FINGER_ON;
        fpi_ssm_jump_to_state(ssm, SCAN_STAGE_WAIT);
        return;

    case GOODIX53XD_CAPTURE_WAIT_FINGER_ON:
        decode_frame(frame, data);
        delta = frame_delta(frame, self->detect_img);

        if (delta > self->finger_threshold) {
            fp_dbg("Finger on, delta: %u", delta);
            fpi_image_device_report_finger_status(img_dev, TRUE);
            self->last_delta = delta;
            self->settle_polls = 0;
            self->capture_mode = GOODIX53XD_CAPTURE_SETTLE;
        }

        fpi_ssm_jump_to_state(ssm, SCAN_STAGE_WAIT);
        return;

    case GOODIX53XD_CAPTURE_SETTLE: {
        decode_frame(frame, data);
        delta = frame_delta(frame, self->detect_img);

        if (delta <= self->finger_threshold) {
            fp_dbg("Finger lifted before settling, delta: %u", delta);
            fpi_image_device_retry_scan(img_dev, FP_DEVICE_RETRY_TOO_SHORT);
            self->capture_mode = GOODIX53XD_CAPTURE_WAIT_FINGER_OFF;
            fpi_ssm_jump_to_state(ssm, SCAN_STAGE_WAIT);
            return;
        }

        const guint change = ABS((int) delta - (int) self->last_delta);
        const gboolean stable =
            change * 100 <= self->last_delta * GOODIX53XD_SETTLE_PERCENT;
        self->last_delta = delta;
        self->settle_polls++;

        if (!stable && self->settle_polls < GOODIX53XD_SETTLE_MAX_POLLS) {
            fpi_ssm_jump_to_state(ssm, SCAN_STAGE_WAIT);
            return;
        }

        fp_dbg("Finger settled, delta: %u after %u polls", delta,
               self->settle_polls);

        if (delta < self->min_contact) {
            fp_dbg("Contact too light, delta %u < %u", delta,
                   self->min_contact);
            fpi_image_device_retry_scan(img_dev, FP_DEVICE_RETRY_CENTER_FINGER);
            self->capture_mode = GOODIX53XD_CAPTURE_WAIT_FINGER_OFF;
            fpi_ssm_jump_to_state(ssm, SCAN_STAGE_WAIT);
            return;
        }

        self->capture_mode = GOODIX53XD_CAPTURE_PRESS;
        self->dac_offset = self->finger_dac_offset;
        fpi_ssm_jump_to_state(ssm, SCAN_STAGE_SWITCH_TO_FDT_MODE);
        return;
    }

    case GOODIX53XD_CAPTURE_PRESS:
        save_frame(self, data);
        if (g_slist_length(self->frames) < GOODIX53XD_CAP_FRAMES) {
            fpi_ssm_jump_to_state(ssm, SCAN_STAGE_SWITCH_TO_FDT_MODE);
            return;
        }

        // The 53xd is a press sensor: the frames all cover the same area,
        // so they are averaged instead of being assembled like a swipe
        Goodix53xdPix averaged[GOODIX53XD_FRAME_SIZE];

        average_frames(self->frames, averaged);
        postprocess_frame(averaged, self->empty_img);
        self->last_amplitude = signal_amplitude(averaged);

        FpImage* img = fp_image_new(GOODIX53XD_IMG_WIDTH, GOODIX53XD_IMG_HEIGHT);
        squash_frame_rows(averaged, img->data);

        dump_press(self, img);
        g_slist_free_full(self->frames, g_free);
        self->frames = NULL;

        // Offline, the strongest signals (hard press, or regular ridges of
        // the finger center) failed to match half of the time. Enrollment
        // keeps them, not to bias the template.
        const FpiDeviceAction action = fpi_device_get_current_action(dev);
        if ((action == FPI_DEVICE_ACTION_VERIFY ||
             action == FPI_DEVICE_ACTION_IDENTIFY) &&
            self->last_amplitude > self->max_amplitude) {
            fp_dbg("Signal amplitude %u > %u, asking for another scan",
                   self->last_amplitude, self->max_amplitude);
            g_object_unref(img);
            fpi_image_device_retry_scan(img_dev, FP_DEVICE_RETRY_REMOVE_FINGER);
        }
        else {
            fpi_image_device_image_captured(img_dev, img);
        }

        self->capture_mode = GOODIX53XD_CAPTURE_WAIT_FINGER_OFF;
        self->dac_offset = self->finger_dac_offset + GOODIX53XD_DETECT_DAC_STEP;
        fpi_ssm_jump_to_state(ssm, SCAN_STAGE_WAIT);
        return;

    case GOODIX53XD_CAPTURE_WAIT_FINGER_OFF:
        decode_frame(frame, data);
        delta = frame_delta(frame, self->detect_img);

        if (delta <= self->finger_threshold) {
            fp_dbg("Finger off, delta: %u", delta);
            // Complete first: reporting the finger off starts the next scan
            fpi_ssm_mark_completed(ssm);
            fpi_image_device_report_finger_status(img_dev, FALSE);
            return;
        }

        fpi_ssm_jump_to_state(ssm, SCAN_STAGE_WAIT);
        return;
    }
}

enum scan_empty_img_state {
    SCAN_EMPTY_NAV0,
    SCAN_EMPTY_GET_IMG,

    SCAN_EMPTY_NUM,
};

static void on_scan_empty_img(FpDevice* dev, guint8* data, guint16 length,
                              gpointer ssm, GError* error)
{
    if (error) {
        fpi_ssm_mark_failed(ssm, error);
        return;
    }
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);
    decode_frame(self->empty_img, data);
    fpi_ssm_next_state(ssm);
}
static void scan_empty_run(FpiSsm* ssm, FpDevice* dev)
{

    switch (fpi_ssm_get_cur_state(ssm)) {
    case SCAN_EMPTY_NAV0:
        goodix_send_nav_0(dev, check_none_cmd, ssm);
        break;

    case SCAN_EMPTY_GET_IMG: {
        FpImageDevice* img_dev = FP_IMAGE_DEVICE(dev);
        FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(img_dev);
        guint8 payload[] = {0x41, 0x03, self->otp[26], 0x00, self->otp[26] - 6, 0x00, self->otp[45], 0x00, self->otp[45] - 4, 0x00};
        goodix_tls_read_image(dev, payload, sizeof(payload), on_scan_empty_img, ssm);
        break;
    }
    }
}

static void scan_empty_img(FpDevice* dev, FpiSsm* ssm)
{
    fpi_ssm_start_subsm(ssm, fpi_ssm_new(dev, scan_empty_run, SCAN_EMPTY_NUM));
}

static void scan_get_img(FpDevice* dev, FpiSsm* ssm)
{
    FpImageDevice* img_dev = FP_IMAGE_DEVICE(dev);
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(img_dev);
    // Four little endian DAC values from the OTP, shifted by the offset of
    // the current capture step
    const int offset = self->dac_offset;
    const int dac[4] = {self->otp[26], self->otp[26] - 6, self->otp[45],
                        self->otp[45] - 4};
    guint8 payload[10] = {0x41, 0x03};

    for (int i = 0; i != 4; ++i) {
        const guint16 value = CLAMP(dac[i] + offset, 0, 0x1ff);
        payload[2 + 2 * i] = value & 0xff;
        payload[3 + 2 * i] = value >> 8;
    }
    goodix_tls_read_image(dev, payload, sizeof(payload), scan_on_read_img, ssm);
}

guint8 fdt_switch_state_mode_53xd[] = {
    0x0d, 0x01, 0x28, 0x01, 0x22, 0x01, 0x28, 0x01,
    0x24, 0x01, 0x91, 0x91, 0x8b, 0x8b, 0x96, 0x96,
    0x91, 0x91, 0x98, 0x98, 0x90, 0x90, 0x92, 0x92,
    0x88, 0x88, 0x00
};

// FDT calibration values come from the OTP of each sensor
static void set_fdt_otp(guint8* fdt, const guint8* otp)
{
    fdt[2] = otp[33];
    fdt[4] = otp[41];
    fdt[6] = otp[42];
    fdt[8] = otp[43];
}

static void scan_run_state(FpiSsm* ssm, FpDevice* dev)
{
    FpImageDevice* img_dev = FP_IMAGE_DEVICE(dev);
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(img_dev);


    switch (fpi_ssm_get_cur_state(ssm)) {

    case SCAN_STAGE_CALIBRATE:
        // Capture frames without finger, using the same sequence as a
        // finger frame (FDT mode, sensor register, image)
        set_fdt_otp(fdt_switch_state_mode_53xd, self->otp);
        const gchar* dac_offset = g_getenv("GOODIX53XD_DAC_OFFSET");
        self->finger_dac_offset =
            dac_offset != NULL ? (int) g_ascii_strtoll(dac_offset, NULL, 10) : 0;
        self->capture_mode = GOODIX53XD_CAPTURE_BACKGROUND;
        self->background_frames = 0;
        self->dac_offset = self->finger_dac_offset + GOODIX53XD_DETECT_DAC_STEP;
        fpi_ssm_jump_to_state(ssm, SCAN_STAGE_SWITCH_TO_FDT_MODE);
        break;

    case SCAN_STAGE_WAIT:
        if (self->deactivating) {
            fpi_ssm_mark_completed(ssm);
            break;
        }
        self->polling = TRUE;
        fpi_ssm_next_state_delayed(ssm, GOODIX53XD_POLL_DELAY);
        break;

    case SCAN_STAGE_SWITCH_TO_FDT_MODE:
        self->polling = FALSE;
        goodix_send_mcu_switch_to_fdt_mode(dev, fdt_switch_state_mode_53xd,
                                           sizeof(fdt_switch_state_mode_53xd), FALSE, NULL,
                                           check_none_cmd, ssm);
        break;

    case SCAN_STAGE_GET_IMG:
        // Register 0x022c, bytes 05 03 on the wire (little endian value)
        goodix_send_write_sensor_register(dev, 0x022c, 0x0305, write_sensor_complete, ssm);
        break;
    }
}

static void write_sensor_complete(FpDevice *dev, gpointer user_data, GError *error)
{
    if (error) {
        fpi_ssm_mark_failed(user_data, error);
        return;
    }
    scan_get_img(dev, user_data);
}

static void scan_complete(FpiSsm* ssm, FpDevice* dev, GError* error)
{
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);

    // A new scan may already have started, only forget this one
    if (self->scan_ssm == ssm) {
        self->scan_ssm = NULL;
        self->polling = FALSE;
    }

    if (error) {
        fp_err("failed to scan: %s (code: %d)", error->message, error->code);
        g_error_free(error);
        return;
    }
    fp_dbg("finished scan");
}

static void scan_start(FpiDeviceGoodixTls53XD* dev)
{
    dev->deactivating = FALSE;
    dev->scan_ssm = fpi_ssm_new(FP_DEVICE(dev), scan_run_state, SCAN_STAGE_NUM);
    fpi_ssm_start(dev->scan_ssm, scan_complete);
}

// ---- SCAN SECTION END ----

// ---- DEV SECTION START ----

/**
 * @brief Drops replies left by a previous process
 * @details A program interrupted during a scan leaves image or TLS data in
 * the reader, which the next activation takes for its command replies. Same
 * as empty_buffer() in goodix-fp-dump: read until it times out.
 *
 * @param dev
 */
static void drain_input(FpDevice* dev)
{
    for (int i = 0; i != GOODIX53XD_DRAIN_MAX_READS; ++i) {
        g_autoptr(FpiUsbTransfer) transfer = fpi_usb_transfer_new(dev);
        g_autoptr(GError) error = NULL;

        fpi_usb_transfer_fill_bulk(transfer, GOODIX_53XD_EP_IN,
                                   GOODIX53XD_DRAIN_SIZE);
        if (!fpi_usb_transfer_submit_sync(transfer, GOODIX53XD_DRAIN_TIMEOUT,
                                          &error)) {
            return;
        }
        fp_dbg("Dropped %zd stale bytes", transfer->actual_length);
    }
}

static void dev_init(FpImageDevice *img_dev) {
  FpDevice *dev = FP_DEVICE(img_dev);
  GError *error = NULL;

  if (!goodix_dev_init(dev, &error)) {
    fpi_image_device_open_complete(img_dev, error);
    return;
  }

  drain_input(dev);
  fpi_image_device_open_complete(img_dev, NULL);
}

static void dev_deinit(FpImageDevice *img_dev) {
  FpDevice *dev = FP_DEVICE(img_dev);
  GError *error = NULL;

  if (goodix_dev_deinit(dev, &error)) {
    fpi_image_device_close_complete(img_dev, error);
    return;
  }

  fpi_image_device_close_complete(img_dev, NULL);
}

static void dev_activate(FpImageDevice *img_dev) {
    FpDevice* dev = FP_DEVICE(img_dev);

    FPI_DEVICE_GOODIXTLS53XD(img_dev)->activate_attempts = 0;
    activate_start(dev, NULL);
}



static void dev_change_state(FpImageDevice* img_dev, FpiImageDeviceState state)
{
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(img_dev);
    G_DEBUG_HERE();

    if (state == FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON) {
        scan_start(self);
    }
}

static void goodix53xd_reset_state(FpiDeviceGoodixTls53XD* self)
{
    // Drop frames of an interrupted press so they don't leak into the next one
    g_slist_free_full(self->frames, g_free);
    self->frames = NULL;
}

static void dev_deactivate(FpImageDevice *img_dev) {
    FpDevice* dev = FP_DEVICE(img_dev);
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(img_dev);

    // Stop the finger polling loop: right away if it is waiting for the next
    // poll, otherwise at its next wait
    self->deactivating = TRUE;
    if (self->scan_ssm != NULL && self->polling) {
        fpi_ssm_cancel_delayed_state_change(self->scan_ssm);
        fpi_ssm_mark_completed(self->scan_ssm);
    }

    goodix_reset_state(dev);
    GError* error = NULL;
    goodix_shutdown_tls(dev, &error);
    goodix53xd_reset_state(FPI_DEVICE_GOODIXTLS53XD(img_dev));
    fpi_image_device_deactivate_complete(img_dev, error);
}

// ---- DEV SECTION END ----

static void fpi_device_goodixtls53xd_init(FpiDeviceGoodixTls53XD* self)
{
    self->frames = NULL;

    const gchar* min_contact = g_getenv("GOODIX53XD_MIN_CONTACT");
    self->min_contact = min_contact != NULL
                            ? (guint) g_ascii_strtoull(min_contact, NULL, 10)
                            : GOODIX53XD_MIN_CONTACT_DELTA;

    const gchar* max_amplitude = g_getenv("GOODIX53XD_MAX_AMPLITUDE");
    self->max_amplitude = max_amplitude != NULL
                              ? (guint) g_ascii_strtoull(max_amplitude, NULL, 10)
                              : GOODIX53XD_MAX_AMPLITUDE;

}

static void fpi_device_goodixtls53xd_class_init(
    FpiDeviceGoodixTls53XDClass *class) {
  FpiDeviceGoodixTlsClass *gx_class = FPI_DEVICE_GOODIXTLS_CLASS(class);
  FpDeviceClass *dev_class = FP_DEVICE_CLASS(class);
  FpImageDeviceClass *img_dev_class = FP_IMAGE_DEVICE_CLASS(class);

  gx_class->interface = GOODIX_53XD_INTERFACE;
  gx_class->ep_in = GOODIX_53XD_EP_IN;
  gx_class->ep_out = GOODIX_53XD_EP_OUT;

  dev_class->id = "goodixtls53xd";
  dev_class->full_name = "Goodix TLS Fingerprint Sensor 53XD";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->id_table = id_table;

  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  dev_class->nr_enroll_stages = GOODIX53XD_ENROLL_STAGES;
  // Finger detection keeps reading images, which exhausts the default
  // temperature model (hot after 3 minutes) during an enrollment. Like the
  // goodixmoc driver, assume the capacitive sensor doesn't heat up.
  dev_class->temp_hot_seconds = -1;

  // NBIS finds 1 or 2 minutiae on the small 53xd images, SIGFM matches
  // SIFT keypoints instead
  img_dev_class->algorithm = FPI_DEVICE_ALGO_SIGFM;
  // Like the Goodix Windows engine: the template keeps up to 50 images,
  // matching scans being added to it
  img_dev_class->sigfm_learn_max = GOODIX53XD_TEMPLATE_MAX;
  img_dev_class->sigfm_min_keypoints = GOODIX53XD_MIN_KEYPOINTS;
  // SIGFM score threshold. Offline, impostors never scored above 3 once the
  // fixed pattern of the sensor is removed by the calibration
  img_dev_class->bz3_threshold = 10;
  img_dev_class->img_width = GOODIX53XD_IMG_WIDTH;
  img_dev_class->img_height = GOODIX53XD_IMG_HEIGHT;

  img_dev_class->img_open = dev_init;
  img_dev_class->img_close = dev_deinit;
  img_dev_class->activate = dev_activate;
  img_dev_class->change_state = dev_change_state;
  img_dev_class->deactivate = dev_deactivate;

  fpi_device_class_auto_initialize_features(dev_class);
}
