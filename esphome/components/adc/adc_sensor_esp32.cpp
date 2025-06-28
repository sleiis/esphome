#ifdef USE_ESP32

#include "adc_sensor.h"
#include "esphome/core/log.h"

#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

namespace esphome {
namespace adc {

static const char *const TAG = "adc.esp32";

// Kanal-Konvertierung
static inline adc_channel_t convert_adc1_to_channel(adc1_channel_t ch) { return static_cast<adc_channel_t>(ch); }
static inline adc_channel_t convert_adc2_to_channel(adc2_channel_t ch) { return static_cast<adc_channel_t>(ch); }

// Kalibrierung
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten,
                                 adc_cali_handle_t *out_handle) {
  adc_cali_handle_t handle = nullptr;
  esp_err_t ret = ESP_FAIL;
  bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  if (!calibrated) {
    ESP_LOGI(TAG, "calibration scheme: Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    calibrated = (ret == ESP_OK);
  }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  if (!calibrated) {
    ESP_LOGI(TAG, "calibration scheme: Line Fitting");
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    calibrated = (ret == ESP_OK);
  }
#endif

  *out_handle = handle;
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Calibration success");
  } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
    ESP_LOGW(TAG, "eFuse not burnt, skipping calibration");
  } else {
    ESP_LOGE(TAG, "Calibration init failed");
  }

  return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  ESP_LOGI(TAG, "Deregister Curve Fitting");
  ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  ESP_LOGI(TAG, "Deregister Line Fitting");
  ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

void ADCSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ADC for '%s'", this->get_name().c_str());

  // Gemeinsamer Handle für alle Instanzen
  static adc_oneshot_unit_handle_t shared_adc1_handle = nullptr;

  if (shared_adc1_handle == nullptr) {
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &shared_adc1_handle));
  }

  this->adc_handle_ = shared_adc1_handle;

  // Initiale Kalibrierung (nicht Kanal-konfig)
  this->calibration_available_ = adc_calibration_init(ADC_UNIT_1, convert_adc1_to_channel(this->channel1_),
                                                      this->attenuation_, &this->cali_handle_);
}

float ADCSensor::sample() {
  if (this->adc_handle_ == nullptr) {
    ESP_LOGE(TAG, "ADC not initialized");
    return NAN;
  }

  // Kanal vor jedem read neu konfigurieren
  adc_oneshot_chan_cfg_t chan_cfg;
  chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
  chan_cfg.atten = this->attenuation_;

  ESP_ERROR_CHECK(adc_oneshot_config_channel(this->adc_handle_, convert_adc1_to_channel(this->channel1_), &chan_cfg));

  int raw = 0;
  int mv = 0;

  if (adc_oneshot_read(this->adc_handle_, convert_adc1_to_channel(this->channel1_), &raw) != ESP_OK) {
    ESP_LOGE(TAG, "ADC read failed");
    return NAN;
  }

  ESP_LOGV(TAG, "'%s': Raw ADC value = %d", this->get_name().c_str(), raw);

  if (this->calibration_available_) {
    if (adc_cali_raw_to_voltage(this->cali_handle_, raw, &mv) != ESP_OK) {
      ESP_LOGW(TAG, "Calibration failed");
      return raw;
    }
    return mv / 1000.0f;  // mV → V
  }

  return raw;
}

void ADCSensor::dump_config() {
  LOG_SENSOR("", "ADC Sensor", this);
  LOG_PIN("  Pin: ", this->pin_);
  switch (this->attenuation_) {
    case ADC_ATTEN_DB_0:
      ESP_LOGCONFIG(TAG, "  Attenuation: 0 dB");
      break;
    case ADC_ATTEN_DB_2_5:
      ESP_LOGCONFIG(TAG, "  Attenuation: 2.5 dB");
      break;
    case ADC_ATTEN_DB_6:
      ESP_LOGCONFIG(TAG, "  Attenuation: 6 dB");
      break;
    case ADC_ATTEN_DB_12_COMPAT:
      ESP_LOGCONFIG(TAG, "  Attenuation: 11/12 dB (compat)");
      break;
    default:
      ESP_LOGCONFIG(TAG, "  Attenuation: unknown");
      break;
  }
  ESP_LOGCONFIG(TAG, "  Samples: %i", this->sample_count_);
}

ADCSensor::~ADCSensor() {
  // shared_adc1_handle wird absichtlich nicht gelöscht
  if (this->calibration_available_) {
    adc_calibration_deinit(this->cali_handle_);
  }
}

}  // namespace adc
}  // namespace esphome

#endif  // USE_ESP32
