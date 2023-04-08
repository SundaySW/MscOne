#ifndef SRC_TNH_HPP_
#define SRC_TNH_HPP_

#include "stm32g4xx_hal.h"
#include <cstring>
#include <functional>

#include "protos_msg.h"
#include "protos_device.h"
#include "base_param.h"
#include "base_device.hpp"

#include "i2c.hpp"
#include "adc.hpp"
#include "tim_ICmode.hpp"
#include "dac.hpp"
#include "eeprom.hpp"
#include "eeprom_24aa02uid.hpp"

#include "onewire_device_pool.h"
#include "onewire_ds2482.h"
#include "onewire_task_provider.h"

#define EEPROM_I2C_ADDR 0x50
#define DS2482_I2C_ADDR 0x18

using namespace Protos;
#define EEPROM_PARAMS_START_ADDR    OneWire::ParamTable::MAX_ROWS_COUNT * OneWire::ParamTable::ROW_SIZE
#define PARAM_NOFCALIB_FIELDS       2
#define EEPROM_PARAM_SIZE           sizeof(float)*PARAM_NOFCALIB_FIELDS


class MscOne : public BaseDevice, public OneWire::TaskProvider
{
public:
    typedef Adc<2> ADC;
    MscOne(DeviceUID::TYPE uidType, uint8_t family, uint8_t addr,
           FDCAN_HandleTypeDef* can, ADC_HandleTypeDef *adc1, I2C_HandleTypeDef *i2c2, DAC_HandleTypeDef *dac1, TIM_HandleTypeDef* timHall)
        : BaseDevice(uidType, family, addr, can)
        ,I2CMaster(I2C(i2c2))
        ,ds2482(I2CMaster, DS2482_I2C_ADDR, *this)
        ,eeprom(&I2CMaster, EEPROM_I2C_ADDR)
        ,OWDevices(OneWire::DevicePool(ds2482))
    {
        AdcA = std::move(ADC(adc1));
        TimIC0 = std::move(Tim_ICMode(timHall, TIM_CHANNEL_1));

        Valve0Ctrl = std::move(DacParam(dac1, DAC_CHANNEL_1));
        Valve0Ctrl.SetId(0xC8);
        Valve0Ctrl.SetCtrlRate(500);
        Valve0Ctrl.SetSendRate(0);
        Valve0Ctrl.SetShort(0);

        Valve1Ctrl = std::move(DacParam(dac1, DAC_CHANNEL_2));
        Valve1Ctrl.SetId(0xC9);
        Valve1Ctrl.SetCtrlRate(500);
        Valve1Ctrl.SetSendRate(0);
        Valve1Ctrl.SetShort(0);

        Preasure0.SetId(0x32);
        Preasure0.SetUpdateRate(1000);
        Preasure0.SetSendRate(2000);

        Preasure1.SetId(0x33);
        Preasure1.SetUpdateRate(1000);
        Preasure1.SetSendRate(2000);

//        using ReturnType = std::invoke_result_t<decltype(makeValueFlowMeter)>;
        hallSensor0.SetId(0x34);
        hallSensor0.SetUpdateRate(1000);
        hallSensor0.SetSendRate(2000);
    }

	void Start()
	{
		uint8_t r_data[6] = {0};
		eeprom.readUID(r_data);
		memcpy(&Uid.Data.I4, r_data, sizeof(uint32_t));
        loadCalibParamsDataFromEEPROM();

        ds2482.Start();
		AdcA.Start();
		TimIC0.Start();
//		Valve0Ctrl.Start();
//		Valve1Ctrl.Start();
		Protos::Device::SendProtosMsg(Protos::BROADCAST, Protos::MSGTYPE_CMDMISC_ANSWER, "12345678", 8);
//		HAL_GPIO_WritePin(W1_SLPZ_GPIO_Port, W1_SLPZ_Pin, GPIO_PIN_SET);
		OWDevices.OnSearch(0x00, OneWire::DEVICE_FAMILY::FAMILY_UNKNOWN);
	}

	bool ProcessTaskResult(const OneWire::Task::Result& task) override
	{
		return OWDevices.ProcessTaskResult(task);
	}

	void OnPoll() override {
        OWDevices.Poll();
        for (auto* param : Params)
            if(param != nullptr) param->Poll();
	};

	void OnTimer(short ms) override{
        for (auto param : Params)
            if(param != nullptr) param->OnTimer(ms);
        OWDevices.OnTimer(ms);
        ds2482.OnDelayTimer();
	};

	void ProcessMessage(const Protos::Msg& msg) override{
        Protos::Msg2 msg2;
        msg2.Pri = msg.Id.Tab[0];
        msg2.Src = msg.GetSenderID();
        msg2.Dst = msg.GetDestID();
        msg2.Type = msg.GetMSGType();
        msg2.Dlc = msg.Dlc;
        for (int i = 0; i < msg.Dlc; i++)
            msg2.Data[i] = msg.Data[i];
        if (msg2.Dst == Address)
            OWDevices.ProcessMessage(msg2);

        auto msgID = msg.GetFieldID();
        switch (msg.GetMSGType()) {
            case MSGTYPE_CMDSPEC:
                break;
            case MSGTYPE_CMDSPEC_ANSWER:
                break;
            case MSGTYPE_PARAM_SET:
            case MSGTYPE_PARAM_REQUEST:
                for (auto param : Params)
                    if(param != nullptr) param->ProcessMessage(msg);
                break;
            case MSGTYPE_PARAM_ANSWER:
                break;
            default:
                break;
        }
	};
    I2C& getI2CMaster(){
        return I2CMaster;
    }
    ADC& getAdcA(){
        return AdcA;
    }

    Tim_ICMode& getTimIC(){
        return TimIC0;
    }

    static void saveCalibParamToEEPROM(char ID, float* data){
        for(int i = 0; i < Params.size(); i++){
            if(Params[i] == nullptr) continue;
            if(Params[i]->GetId() == ID){
                char buffer[EEPROM_PARAM_SIZE];
                int Offset = EEPROM_PARAMS_START_ADDR + EEPROM_PARAM_SIZE*i;
                memcpy(buffer, data, EEPROM_PARAM_SIZE);
                eeprom_write_block(Offset, buffer, EEPROM_PARAM_SIZE);
            }
        }
    }

    void loadCalibParamsDataFromEEPROM(){
        for(int i = 0; i < Params.size(); i++){
            if(Params[i] == nullptr) continue;
            CalibrParam* calibPtr = nullptr;
            Params[i]->QueryInterface(IID_CALIBRATEABLE, (void*&)calibPtr);
            if(calibPtr == nullptr) continue;
            char buffer[EEPROM_PARAM_SIZE];
            int Offset = EEPROM_PARAMS_START_ADDR + EEPROM_PARAM_SIZE * i;
            eeprom_read_block(Offset, buffer, EEPROM_PARAM_SIZE);
            float calibData[PARAM_NOFCALIB_FIELDS];
            memcpy(calibData, buffer, EEPROM_PARAM_SIZE);
            calibPtr->SetOffset(calibData[0]);
            calibPtr->SetMult(calibData[1]);
        }
    }

private:
    I2C I2CMaster;
    inline static ADC AdcA;
    inline static Tim_ICMode TimIC0;
    inline static AdcParam Preasure0 = AdcParam(&AdcA, 0, &saveCalibParamToEEPROM);
    inline static AdcParam Preasure1 = AdcParam(&AdcA, 1, &saveCalibParamToEEPROM);
    inline static Tim_ICmParam hallSensor0 = Tim_ICmParam(&TimIC0, &saveCalibParamToEEPROM);
    inline static DacParam Valve0Ctrl;
    inline static DacParam Valve1Ctrl;
    inline static constexpr int PARAM_CNT = 5;
    inline static constexpr auto Params{[]() constexpr{
        std::array<BaseParam*, PARAM_CNT> result{};
        int pCount = PARAM_CNT-1;
        result[pCount--] =  (BaseParam*)&Valve0Ctrl;
        result[pCount--] =  (BaseParam*)&Valve1Ctrl;
        result[pCount--] =  (BaseParam*)&Preasure0;
        result[pCount--] =  (BaseParam*)&Preasure1;
        result[pCount] =    (BaseParam*)&hallSensor0;
        return result;
    }()};
    Eeprom24AAUID eeprom;
    OneWire::DS2482 ds2482;
    OneWire::DevicePool OWDevices;
};
#endif /* SRC_TNH_HPP_ */
