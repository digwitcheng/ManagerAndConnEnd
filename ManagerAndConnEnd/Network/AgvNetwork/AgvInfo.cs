using AGVSocket.Network.EnumType;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network
{
    class AgvInfo
    {
        private CurrentLocation curLocation;
        private AgvMotionState agvMotion;
        private TrayState tray;
        private byte electricity;
        private ObstacleAvoidSwitch obstacleSwitch;
        private AlarmState alarm;
        private OrderExecState orderExec;

        #region Properties
        public CurrentLocation CurLocation { get { return curLocation; } }
        public AgvMotionState AgvMotion { get { return agvMotion; } }
        public TrayState Tray { get { return tray; } }
        public byte Electricity { get { return electricity; } }
        public ObstacleAvoidSwitch ObstacleSwitch { get { return obstacleSwitch; } }
        public AlarmState Alarm { get { return alarm; } }
        public OrderExecState OrderExec { get { return orderExec; } }

        #endregion

        public AgvInfo(byte[] data, int offset)
        {
            this.curLocation = new CurrentLocation(data, ref offset);
            this.agvMotion = (AgvMotionState)data[offset++];
            this.tray =(TrayState) data[offset++];
            this.electricity = data[offset++];
            this.obstacleSwitch =(ObstacleAvoidSwitch) data[offset++];
            this.alarm =(AlarmState) data[offset++];
            this.orderExec =(OrderExecState) data[offset++];
        }

    }
}
