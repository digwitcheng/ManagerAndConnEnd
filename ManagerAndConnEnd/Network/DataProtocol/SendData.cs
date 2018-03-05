using AGV_V1._0.Agv;
using Agv.PathPlanning;
using Newtonsoft.Json;
using Newtonsoft.Json.Converters;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0
{
    class SendData
    {
       public SendData()
        {

        }
       public SendData(int num, int BeginX, int BeginY,int EndX,int EndY)
        {
            this.Num = num;
            this.BeginX = BeginX;
            this.BeginY = BeginY;
            this.EndX = EndX;
            this.EndY = EndY;
        }
       public SendData(int num, int BeginX, int BeginY)
       {
           this.Num = num;
           this.BeginX = BeginX;
           this.BeginY = BeginY;
       }

        //判断小车是否到终点
        private bool arrive;

        public bool Arrive
        {
            get { return arrive; }
            set { arrive = value; }
        }
        //小车编号
        public int Num { get; set; }
        //小车的状态
        private State state;
         [JsonConverter(typeof(StringEnumConverter))]
        public State State
        {
            get { return state; }
            set { state = value; }
        }
        //小车的方向
        private Direction direction;
         [JsonConverter(typeof(StringEnumConverter))]
        public Direction Direction
        {
            get { return direction; }
            set { direction = value; }
        }
        //小车的电量
        private int battery;

        public int Battery
        {
            get { return battery; }
            set { battery = value; }
        }
        public int BeginX { get; set; }
        public int BeginY { get; set; }
        public int EndX { get; set; }
        public int EndY { get; set; }
        public int DestX { get; set; }
        public int DestY { get; set; }

        private string startloc;
        public string StartLoc
        {
            get { return startloc; }
            set { startloc = value; }
        }

        private string endloc;
        public string EndLoc
        {
            get { return endloc; }
            set { endloc = value; }
        }
    }
}
