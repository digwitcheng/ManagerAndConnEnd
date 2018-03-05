using AGVSocket.Network.EnumType;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network.Packet
{
    class SysResponsePacket:SendBasePacket
    {
        private byte responseType;   //需要应答报文类型
        private ResponseState responseState;  //需要应答报文状态

       // public byte ResponseType { set; }
       // public byte ResponseState { set; }

        public SysResponsePacket(byte serialNum, ushort agvId, byte type,ResponseState state)
        {
            this.Header = 0xAA55;
            this.Len = NeedLen();
            this.SerialNum = serialNum;
            this.Type = (byte)PacketType.SysResponse;
            this.AgvId = agvId;
            this.responseType = type;
            this.responseState = state;
        }
        protected override byte NeedLen()
        {
            return 10;
        }

        public override byte[] GetBytes()
        {
            byte[] packData = new byte[NeedLen()];
            int offset=FillBytes(packData);

            packData[offset++] = responseType;
            packData[offset++] = (byte)responseState;
            packData[NeedLen() - 1] = CaculateCheckSum(packData);
            this.CheckSum = packData[NeedLen() - 1];
            return packData;
        }
    }
}
