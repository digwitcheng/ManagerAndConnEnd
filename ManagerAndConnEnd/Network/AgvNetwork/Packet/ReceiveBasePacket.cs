using AGV_V1._0.NLog;
using AGVSocket.Network.EnumType;
using AGVSocket.Network.MyException;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network.Packet
{
    abstract class ReceiveBasePacket : BasePacket
    {
        private ResponseState isCheckSumCorrect = ResponseState.Correct;
        public ResponseState IsCheckSumCorrect
        {
            get { return isCheckSumCorrect; }
            protected set { isCheckSumCorrect = value; }
        }

        /// <summary>
        /// 初始化父类中的成员数据
        /// </summary>
        /// <param name="ClassTpye">子类的名称</param>
        /// <param name="data">数据</param>
        protected ReceiveBasePacket(string ClassTpye, byte[] data)
        {

            if (data == null || data.Length < NeedLen())
            {
                throw new PacketException(ClassTpye, ExceptionCode.DataMiss);
            }
            if (data[NeedLen() - 1] != CaculateCheckSum(data))
            {
                throw new PacketException(ClassTpye, ExceptionCode.CheckSumError);
            }
            int offset = 0;
            this.Header = MyBitConverter.ToUInt16(data, ref offset);
            this.Len = data[offset++];
            this.SerialNum = data[offset++];
            this.Type = data[offset++];
            this.AgvId = MyBitConverter.ToUInt16(data, ref offset);
            this.CheckSum = data[NeedLen() - 1];

        }
        protected ReceiveBasePacket() { }
        public abstract void Receive();
        public abstract byte NeedLen();

        public static ReceiveBasePacket Factory(PacketType type, byte[] data)
        {
            try
            {
                switch (type)
                {
                    case PacketType.DoneReply:
                        return new DoneReplyPacket(data);
                    case PacketType.AgvInfo:
                        return new AgvInfoPacket(data);
                    case PacketType.AgvResponse:
                        return new AgvResponsePacket(data);
                    default:
                        // return new ErrorPacket(data);
                       throw new PacketException("factory", ExceptionCode.NonsupportType);
                }
            }
            catch (PacketException pe)
            {
                //Send(new SysResponsePacket(1,buffers[));
                // throw;
                if (pe.Code == ExceptionCode.CheckSumError)
                {
                    return new ErrorPacket(data);
                }
                else if (pe.Code == ExceptionCode.DataMiss && data.Length >= 7)
                {
                    return new ErrorPacket(data);
                }
                else
                {
                    throw;
                }
            }
            catch (Exception ex)
            {
                Logs.Error("未知错误:" + ex);
                throw;
               // return new ErrorPacket(data);
            }

        }

        protected byte CaculateCheckSum(byte[] data)
        {
            uint checkSum = 0;
            for (int i = 0; i < data[2] - 1; i++)
            {
                checkSum += data[i];
            }
            return (byte)(checkSum & 0x7f);
        }
        public void ReceiveResponse()
        {
            AgvServerManager.Instance.Send(new SysResponsePacket(this.SerialNum, this.AgvId, this.Type, this.IsCheckSumCorrect));
        }
    }
}
