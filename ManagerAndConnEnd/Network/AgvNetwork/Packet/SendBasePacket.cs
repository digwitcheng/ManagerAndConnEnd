using Cowboy.Sockets;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network.Packet
{
   abstract class SendBasePacket:BasePacket
    {



        /// <summary>
        /// 把父类的成员数据转换成字节填充到数组中
        /// </summary>
        /// <param name="needFill">需要填充的数组</param>
        /// <returns>下标偏移的位置</returns>
        protected int FillBytes(byte[] needFill)
        {
            int offset = 0;
            Buffer.BlockCopy(MyBitConverter.GetBytes(this.Header), 0, needFill, offset, 2);
            offset += 2;
            needFill[offset++] = this.Len;
            needFill[offset++] = this.SerialNum;
            needFill[offset++] = this.Type;
            Buffer.BlockCopy(MyBitConverter.GetBytes(this.AgvId), 0, needFill, offset, 2);
            offset += 2;            
            return offset;
        }
       protected byte CaculateCheckSum(byte[] data)
        {
            uint checkSum = 0;
            for (int i = 0; i < data.Length - 1; i++)
            {
                checkSum += data[i];
            }
            return (byte)(checkSum & 0x7f);
        }
       protected abstract byte NeedLen();
       public abstract byte[] GetBytes();
       public void Send(TcpSocketServer server)
       {
           server.Broadcast(GetBytes());
       }
    }
}
