using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network.EnumType
{
    enum AlarmState
    {
       
       Normal= 0x00,  // 正常状态
       HardwareLimit= 0x01,  // 硬件限位报警
       CommunicationFault= 0x02,// 通信故障报警
       ScanNone= 0x03,  // 没扫描到二维码报警
       OpticalBand=0x04,  // 光带故障
       BeltFault= 0x05,  // 皮带故障
       MotorFault= 0x06,  // 电机故障
       Obstacle =0X07  // 障碍物报警   


    }
}
