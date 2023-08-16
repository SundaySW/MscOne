#ifndef SRC_MSCONE_HPP_
#define SRC_MSCONE_HPP_

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
#define EEPROM_PARAMS_START_ADDR    OneWire::ParamTable::MAX_ROWS_COUNT * OneWire::ParamTable::ROW_SIZE
#define PARAM_NOFCALIB_FIELDS       2
#define EEPROM_PARAM_SIZE           (sizeof(float)*PARAM_NOFCALIB_FIELDS)

using namespace Protos;

class MscOne : public BaseDevice, public OneWire::TaskProvider
{
public:
    using ADCc3 = Adc<3>;
    using ADCc2 = Adc<2>;
    MscOne() = delete;
    MscOne(MscOne&) = delete;
    MscOne(MscOne&&) = delete;


    static MscOne& getInstance(){
        static auto self = MscOne(DeviceUID::TYPE_MICROCHIP, 0x01, 0x20, &hfdcan1);
        return self;
    }
    void initPerf(ADC_HandleTypeDef *adc1,
                  ADC_HandleTypeDef *adc2,
                  I2C_HandleTypeDef *i2c2,
                  DAC_HandleTypeDef *dac1,
                  TIM_HandleTypeDef* timHall0,
                  TIM_HandleTypeDef* timHall1)
    {
        I2CMaster = I2C(i2c2);
        AdcA1 = std::move(ADCc3(adc1));
        AdcA2 = std::move(ADCc2(adc2));
        TimIC0 = std::move(Tim_ICMode(timHall0, TIM_CHANNEL_1));
        TimIC1 = std::move(Tim_ICMode(timHall1, TIM_CHANNEL_1));

        Valve0Ctrl = std::move(DacParam(dac1, DAC_CHANNEL_1));
        Valve0Ctrl.SetId(0xC1);
        Valve0Ctrl.SetCtrlRate(500);
        Valve0Ctrl.SetSendRate(0);
        Valve0Ctrl.SetShort(0);

        Valve1Ctrl = std::move(DacParam(dac1, DAC_CHANNEL_2));
        Valve1Ctrl.SetId(0xC2);
        Valve1Ctrl.SetCtrlRate(500);
        Valve1Ctrl.SetSendRate(0);
        Valve1Ctrl.SetShort(0);

        Preasure1.SetId(0x31);
        Preasure1.SetUpdateRate(1000);
        Preasure1.SetSendRate(2000);

        Preasure2.SetId(0x32);
        Preasure2.SetUpdateRate(1000);
        Preasure2.SetSendRate(2000);

        Preasure3.SetId(0x33);
        Preasure3.SetUpdateRate(1000);
        Preasure3.SetSendRate(2000);

        Preasure4.SetId(0x34);
        Preasure4.SetUpdateRate(1000);
        Preasure4.SetSendRate(2000);

        Preasure5.SetId(0x35);
        Preasure5.SetUpdateRate(1000);
        Preasure5.SetSendRate(2000);

//      using ReturnType = std::invoke_result_t<decltype(makeValueFlowMeter)>;
        hallSensor0.SetId(0x41);
        hallSensor0.SetUpdateRate(1000);
        hallSensor0.SetSendRate(2000);

        hallSensor1.SetId(0x42);
        hallSensor1.SetUpdateRate(1000);
        hallSensor1.SetSendRate(2000);
    }

	void Start()
	{
		readUID();
        loadCalibParamsDataFromEEPROM();
        ds2482.Start();
		AdcA1.Start();
        AdcA2.Start();
		TimIC0.Start();
        TimIC1.Start();
		Valve0Ctrl.Start();
		Valve1Ctrl.Start();
//		Protos::Device::SendProtosMsg(Protos::BROADCAST, Protos::MSGTYPE_CMDMISC_ANSWER, "12345678", 8);
		OWDevices.OnSearch(0x00, OneWire::DEVICE_FAMILY::FAMILY_UNKNOWN);
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

    static void loadCalibParamsDataFromEEPROM(){
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

    void readUID(){
        uint8_t r_data[6] = {0};
        eeprom.readUID(r_data);
        memcpy(&Uid.Data.I4, r_data, sizeof(uint32_t));
    }

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

    I2C& getI2CMaster(){
        return I2CMaster;
    }

    static void processADCCallback(ADC_HandleTypeDef* hadc){
        if (hadc == AdcA1.getHandler())
            AdcA1.OnCallback();
        else if(hadc == AdcA2.getHandler())
            AdcA2.OnCallback();
    }

    static void processTimCallBack(TIM_HandleTypeDef* htim){
        if(htim == TimIC0.getHTim())
            TimIC0.OnCollBack();
    }

private:
    MscOne(DeviceUID::TYPE uidType, uint8_t family, uint8_t addr, FDCAN_HandleTypeDef* can)
            : BaseDevice(uidType, family, addr, can)
            ,ds2482(I2CMaster, DS2482_I2C_ADDR, *this)
            ,eeprom(&I2CMaster, EEPROM_I2C_ADDR)
            ,OWDevices(OneWire::DevicePool(ds2482))
    {}

    I2C I2CMaster;
    inline static ADCc3 AdcA1;
    inline static ADCc2 AdcA2;
    inline static Tim_ICMode TimIC0;
    inline static Tim_ICMode TimIC1;
    inline static AdcParam Preasure1{AdcParam(&AdcA1, 0, &saveCalibParamToEEPROM)};
    inline static AdcParam Preasure2{AdcParam(&AdcA1, 1, &saveCalibParamToEEPROM)};
    inline static AdcParam Preasure3{AdcParam(&AdcA1, 2, &saveCalibParamToEEPROM)};
    inline static AdcParam Preasure4{AdcParam(&AdcA2, 0, &saveCalibParamToEEPROM)};
    inline static AdcParam Preasure5{AdcParam(&AdcA2, 1, &saveCalibParamToEEPROM)};
    inline static Tim_ICmParam hallSensor0{Tim_ICmParam(&TimIC0, &saveCalibParamToEEPROM)};
    inline static Tim_ICmParam hallSensor1{Tim_ICmParam(&TimIC1, &saveCalibParamToEEPROM)};
    inline static DacParam Valve0Ctrl;
    inline static DacParam Valve1Ctrl;
    inline static constexpr int PARAM_CNT = 9;
    inline static constexpr auto Params{[]() constexpr{
        std::array<BaseParam*, PARAM_CNT> result{};
        int pCount = PARAM_CNT-1;
        result[pCount--] = (BaseParam*)&Valve0Ctrl;
        result[pCount--] = (BaseParam*)&Valve1Ctrl;
        result[pCount--] = (BaseParam*)&Preasure1;
        result[pCount--] = (BaseParam*)&Preasure2;
        result[pCount--] = (BaseParam*)&Preasure3;
        result[pCount--] = (BaseParam*)&Preasure4;
        result[pCount--] = (BaseParam*)&Preasure5;
        result[pCount--] = (BaseParam*)&hallSensor0;
        result[pCount]   = (BaseParam*)&hallSensor1;
        return result;
    }()};
    Eeprom24AAUID eeprom;
    OneWire::DS2482 ds2482;
    OneWire::DevicePool OWDevices;
};

void SendMsg(char dest, char msgType, const char *data, char dlc)
{
    MscOne::getInstance().SendProtosMsg(dest, msgType, data, dlc);
}
void SendMsg(char dest, char msgType, char data0, char data1)
{
    char buf[8];
    buf[0] = data0;
    buf[1] = data1;
    MscOne::getInstance().SendProtosMsg(dest, msgType, buf, 2);
}

#endif /* SRC_MSCONE_HPP_ */