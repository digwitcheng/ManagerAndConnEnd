using System;
using System.Linq;
using System.Text;
using System.Drawing;
using AGV_V1._0;
using System.Collections.Generic;
using Astar;



namespace AGV_V1
{



    class MapNode
    {

        //  public   enum MapNodeType1 { img_Belt,img_Mid ,img_Road,img_Destination,img_ChargeStation,img_Obstacle,img_Scanner};

        private int x;               //节点的横坐标    
        private int y;               //节点的纵坐标     


        private int nodeCanUsed = -1;   //节点是否被占用,值表示被编号为几的小车占用，-1表示没有被占用
        private static readonly object canUseLock = new object();
        public int NodeCanUsed
        {
            get
            {
                lock (canUseLock)
                {
                    return nodeCanUsed;
                }
            }
            set
            {
                lock (canUseLock)
                {
                    nodeCanUsed = value;
                }
            }
        }

        // public int LockNode = -1;  //-1节点没有被锁定，大于-1表示被锁定

        private int traCongesIntensity;//traffic congestion intensity 节点拥堵程度

        public int TraCongesIntensity
        {
            get
            {
                return traCongesIntensity;
            }
            set
            {
                if (value < 0)
                {
                    value = 0;
                }
                traCongesIntensity = value;
            }
        }
        // public int PassDifficulty { get { return passDifficulty; } set { passDifficulty = value; } }

        //节点通行难度,数值越大表示越难通行,默认为2，中等通行难度，不可通行用一个非常大的数表示（100）
        public const int UNABLE_PASS = 100;
        public const int MAX_ABLE_PASS = 10;
        public const int DEFAULT_DIFFICULTY = 2;
        private int upDifficulty = DEFAULT_DIFFICULTY;
        private int downDifficulty = DEFAULT_DIFFICULTY;
        private int leftDifficulty = DEFAULT_DIFFICULTY;
        private int rightDifficulty = DEFAULT_DIFFICULTY;


        public int UpDifficulty { get { return upDifficulty; } set { upDifficulty = value; } }
        public int DownDifficulty { get { return downDifficulty; } set { downDifficulty = value; } }
        public int LeftDifficulty { get { return leftDifficulty; } set { leftDifficulty = value; } }
        public int RightDifficulty { get { return rightDifficulty; } set { rightDifficulty = value; } }


        public bool IsAbleCross    //节点可不可走,true表示可走，false表示不可走 
        {
            get;
            set;
        }
        public MapNodeType Type    //节点类型
        {
            get;
            set;
        }

        public int Id  //节点的编号
        { get; private set; }
        public int X
        {
            get { return x; }
            set { x = value; }
        }
        public int Y
        {
            get { return y; }
            set { y = value; }
        }

        /// <summary>
        /// 含参构造函数
        /// </summary>
        /// <param name="id"></param>
        /// <param name="reachable"></param>
        public MapNode(int id, bool node_type)
        {
            this.Id = id;
            this.IsAbleCross = node_type;

        }

        /// <summary>
        /// 无参构造函数
        /// </summary>
        public MapNode()
        { }
    }
}
