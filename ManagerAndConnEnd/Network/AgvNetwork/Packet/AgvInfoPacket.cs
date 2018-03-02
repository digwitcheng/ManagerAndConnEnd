using AGV_V1._0;
using AGV_V1._0.Queue;
using AGVSocket.Network.EnumType;
using AGVSocket.Network.MyException;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network.Packet
{
    class AgvInfoPacket:ReceiveBasePacket
    {
        private AgvInfo info;//小车详情
        //private const byte NEEDLEN = 24;
        public AgvInfo Info { get; set; }
        
        public AgvInfoPacket(byte[] data)
            : base("AgvInfoPacket", data)
        {
            this.info=new AgvInfo(data,7);
            this.CheckSum = data[NeedLen() - 1];
           
        }

        public override void Receive()
        {
            //string str=string.Format("update Vehicle set CurX={0} ,CurY={1}, DesX={2}, DesY={3} where Id={4}",
            //    info.CurLocation.CurNode.X,info.CurLocation.CurNode.Y,
            //    info.CurLocation.DesNode.X,info.CurLocation.DesNode.Y,
            //    this.AgvId);
            //CmdTxtQueue.Instance.Enqueue(str);

            //Debug.WriteLine(string.Format("小车{0}:当前位置({1},{2})", this.AgvId, info.CurLocation.CurNode.X, this.info.CurLocation.CurNode.Y));
            //Debug.WriteLine(string.Format("小车{0}:目的位置({1},{2})", this.AgvId, info.CurLocation.DesNode.X, this.info.CurLocation.DesNode.Y));
            //Debug.WriteLine("-----------------");

            VehicleManager.Instance.AddOrUpdate(this.AgvId, this.info);

        }

        public override byte NeedLen()
        {
            return 35;
        }
    }
}
