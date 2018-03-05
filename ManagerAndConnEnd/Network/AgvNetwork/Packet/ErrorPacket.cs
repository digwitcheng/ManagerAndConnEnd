using AGVSocket.Network.EnumType;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network.Packet
{
    class ErrorPacket:ReceiveBasePacket
    {
        public ErrorPacket(byte[] data)
        {           
            int offset = 0;
            this.Header = MyBitConverter.ToUInt16(data, ref offset);
            this.Len = data[offset++];
            this.SerialNum = data[offset++];
            this.Type = data[offset++];
            this.AgvId = MyBitConverter.ToUInt16(data, ref offset);
            this.IsCheckSumCorrect = ResponseState.Error;
            
        }

        public override void Receive()
        {
            Debug.WriteLine("错误报文!小车{0}，报文类型{1}");
        }

        public override byte NeedLen()
        {
            return 10;
        }
    }
}
