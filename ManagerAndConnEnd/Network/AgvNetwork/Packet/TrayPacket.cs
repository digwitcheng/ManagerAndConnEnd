using AGVSocket.Network.EnumType;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network.Packet
{
    class TrayPacket:SendBasePacket
    {
        private TrayMotion trayInfo; //托盘控制信息
       // public TrayAction TrayInfo { set; }
        protected override byte NeedLen()
        {
            return 9;
        }
        public TrayPacket(byte serialNum, ushort agvId, TrayMotion trayInfo)
        {
            this.Header = 0xAA55;
            this.Len = NeedLen();
            this.SerialNum = serialNum;
            this.Type = (byte)PacketType.Tray;
            this.AgvId = agvId;
            this.trayInfo = trayInfo;
        }
        public override byte[] GetBytes()
        {
            byte[] packData = new byte[NeedLen()];
            int offset=FillBytes(packData);

            packData[offset++] = (byte)trayInfo;
            packData[NeedLen() - 1] = CaculateCheckSum(packData);
            this.CheckSum = packData[NeedLen() - 1];

            return packData;
        }
    }
}
