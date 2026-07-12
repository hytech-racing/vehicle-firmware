#ifndef THERMISTOR
#define THERMISTOR
#include "stdint.h"
#include <math.h>
//so in case we ever have new thermistor types, I will have a thermistor class and a thermistor wrapper


const float DEFAULT_THERM_BETA = 3606.05;
const uint16_t DEFAULT_ADC_SATUR = 4095;
const float DEFAULT_ZERO_KELVIN = 273.15;
const float DEFAULT_T0_CELCIUS = 25;
const float DEFAULT_R_NOM = 10000;
const float DEFAULT_R0 = 8350;

typedef struct thermistor_params {
    float _beta;
    uint16_t _adc_saturation;
    float _zero_kelvin;
    float _t0_celcius;
    float _r_nom;
    float _r0;
} thermistor_params;

class Thermistor {
    float _beta;
    uint16_t _adc_saturation;
    float _zero_kelvin;
    float _t0_celcius;
    float _r_nom;
    float _r0;

    uint16_t _raw;
    float _temp_celcius;
public:

    Thermistor(const float beta, const uint16_t adc_saturation, const float zero_kelvin, const float t0_celcius, const float r_nom, const float r0)
    {
        _beta = beta;
        _adc_saturation = adc_saturation;
        _zero_kelvin = zero_kelvin;
        _t0_celcius = t0_celcius;
        _r_nom = r_nom;
        _r0 = r0;
        _raw = 0;
        _temp_celcius = 0;
    }
    Thermistor()
        : Thermistor(DEFAULT_THERM_BETA, DEFAULT_ADC_SATUR, DEFAULT_ZERO_KELVIN, DEFAULT_T0_CELCIUS, DEFAULT_R_NOM, DEFAULT_R0) {
    }
    Thermistor(thermistor_params& params) : Thermistor(params._beta, params._adc_saturation, params._zero_kelvin, params._t0_celcius, params._r_nom, params._r0){

    }
    
    void setParams(thermistor_params& params) {
        _beta = params._beta;
        _adc_saturation = params._adc_saturation;
        _zero_kelvin = params._zero_kelvin;
        _t0_celcius = params._t0_celcius;
        _r_nom = params._r_nom;
        _r0 = params._r0;
    }

    virtual float convert(const uint16_t raw) //can be overriden in the case its a diff thermistor type.
    {
        float _t0_kelvin = _t0_celcius + _zero_kelvin;
        float resistance;
        float temp_kelvin;
        float temp_celcius;

        _raw = raw;
        resistance = _r0 * raw / (_adc_saturation - raw);
        temp_kelvin = 1/ (1/_t0_kelvin + log(resistance/_r_nom)/_beta);
        temp_celcius = temp_kelvin - _zero_kelvin;
        _temp_celcius = temp_celcius;
        return _temp_celcius;
    }

    float getTemp() {
        return _temp_celcius;
    }
};

template <int NUM_THERMISTORS>
class Thermistors
{
    protected:
    Thermistor thermistors_[NUM_THERMISTORS];
    public:

    Thermistors()
    {
        thermistors_init();
    }

    void thermistors_init() {
        for (int i = 0; i < NUM_THERMISTORS; i++)
        {
            thermistors_[i] = Thermistor();
        }
    }
    const Thermistor& get(int channel) 
    {
        return thermistors_[channel];
    }
    
    void tick(unsigned long curr_millis);

};

#endif