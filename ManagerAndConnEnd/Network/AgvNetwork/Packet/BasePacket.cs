using AGV;
using Cowboy.Sockets;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network.Packet
{
    abstract class BasePacket
    {
        private ushort header = 0xAA55;  //报文头
        private byte len;                //报文长度(字节?)
        private byte serialNum;          //报文序号 0-255 循环指令序号 
        private byte type;               //报文类型
        private ushort agvId;            //AGV 编号
        private byte checkSum;           //校验和 前面所有内容按字节累加，再与上 0x7F

        public ushort Header
        {
            get { return header; }
            protected set { header = value; }
        }
        public byte Len
        {
            get { return len; }
            protected set { len = value; }
        }
        public byte SerialNum
        {
            get { return serialNum; }
            protected set { serialNum = value; }
        }
        public byte Type
        {
            get { return type; }
            protected set { type = value; }
        }
        public ushort AgvId
        {
            get { return agvId; }
            protected set { agvId = value; }
        }
        public byte CheckSum
        {
            get { return checkSum; }
            protected set { checkSum = value; }
        }

        //public abstract BasePacket Create(string msg);
        //public abstract void Send(string sessionKey, TcpSocketServer server);

        //public abstract void Receive();
        //protected abstract byte GetCheckSum();
        // protected abstract byte[] SetBytes();
        //public event EventHandler<PackMessageEventArgs> ShowMessage;
        //public event EventHandler<PackMessageEventArgs> DataMessage;
        //public event EventHandler<PackMessageEventArgs> ErrorMessage;
        //public delegate void ReLoadDele();
        //public ReLoadDele ReLoad;

    }
}
