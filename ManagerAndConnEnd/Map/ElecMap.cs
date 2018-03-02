using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.Data;
using System.Windows.Forms;
using AGV_V1._0.Properties;
using Astar;
using System.Xml;
using AGV_V1._0.Algorithm;
using AGV_V1._0.NLog;
using AGV_V1._0.Util;
using System.IO;

namespace AGV_V1._0
{

    class ElecMap
    {

        //private static Image img_Belt = Resources.Belt;
        //private static Image img_Mid = Resources.Mid;
        //private static Image img_Road = Resources.Road;
        //private static Image img_Destination = Resources.Destination;
        //private static Image img_ChargeStation = Resources.ChargeStation;
        //private static Image img_Obstacle = Resources.Obstacle;
        //private static Image img_Scanner = Resources.Scanner;

        private bool isInited = false;
        private static ElecMap instance;
        public static ElecMap Instance
        {
            get
            {
                if (null == instance)
                {
                    instance = new ElecMap();
                }
                return instance;
            }
        }

        private ElecMap()
        {
            InitialElc();
        }
        //public  int Width; //电子地图的宽度
        //public  int Height; //电子地图的长度
        public int HeightNum//电子地图的长被分割的个数
        { get; private set; }
        public int WidthNum//电子地图的宽被分割的个数  
        { get; private set; }
        public MapNode[,] mapnode;
        private List<MyPoint> scanner = new List<MyPoint>();
        private List<MyPoint> queueEntra = new List<MyPoint>();
        private List<MyPoint> queuingArea = new List<MyPoint>();
        private List<MyPoint> restArea = new List<MyPoint>();


        public List<MyPoint> GetRestArea()
        {
            return restArea;

        }
        public List<MyPoint> GetScanner()
        {
            return scanner;

        }
        
        public bool IsScanner(int x, int y)
        {
            for (int i = 0; i < scanner.Count; i++)
            {
                if (scanner[i].X == x && scanner[i].Y == y)
                {
                    return true;
                }
            }
            return false;
        }
        public bool IsQueueEntra(int x, int y)
        {
            for (int i = 0; i < queueEntra.Count; i++)
            {
                if (queueEntra[i].X == x && queueEntra[i].Y == y)
                {
                    return true;
                }
            }
            return false;
        }
        public bool IsSpecialArea(int x,int y)
        {
            for (int i = 0; i < scanner.Count; i++)
            {
                if (scanner[i].X == x && scanner[i].Y == y)
                {
                    return true;
                }
            }
            for (int i = 0; i < queueEntra.Count; i++)
            {
                if (queueEntra[i].X == x && queueEntra[i].Y == y)
                {
                    return true;
                }
            }
            for (int i = 0; i < queuingArea.Count; i++)
            {
                if (queuingArea[i].X == x && queuingArea[i].Y == y)
                {
                    return true;
                }
            }
            return false;
        }
        void LoadMapFile()
        {
            try
            {
                FileUtil.LoadMapXml(); //初始化XML配置文件
            }
            catch (FileNotFoundException ex)
            {
                Logs.Error("文件未找到:" + ex);
            }
            catch (FileLoadException ex)
            {
                Logs.Error("文件加载异常：" + ex);
            }
            catch (Exception ex)
            {
                MessageBox.Show("地图文件加载失败"+ex);
                Logs.Warn("地图文件加载失败"+ex);
            }
        }
        //}
        /// <summary>
        /// 初始化电子地图，
        /// </summary>
        public void InitialElc()
        {
            if (isInited)
            {
                return;
            }
            isInited = true;
            LoadMapFile();
            HeightNum = ConstDefine.g_HeightNum;
            WidthNum = ConstDefine.g_WidthNum;

            mapnode = new MapNode[HeightNum, WidthNum];
            int Node_number = 0;    //节点编号

            //循环变量
            int i = 0;
            int j = 0;

            ////横纵坐标的控制变量
            //int point_x, point_y;

            //节点类型
            bool point_type = true;

            for (i = 0; i < HeightNum; i++)
            {
                for (j = 0; j < WidthNum; j++)
                {
                    mapnode[i, j] = new MapNode( Node_number, point_type);
                    Node_number++;

                }
            }

            ////铺路，将所有的空间都置为道路

            //for (i = 0; i < HeightNum; i++)
            //{
            //    point_x = ConstDefine.BLANK_X;
            //    for (j = 0; j < WidthNum; j++)
            //    {
            //        mapnode[i, j].oth = img_Road;// drawArrow(i, j); //
            //    }
            //}

            //继承上一次的场景设置，显示场地分布
            if (null == FileUtil.xmlfile)
            {
                Logs.Error("地图文件读取失败！");
                return;
            }
            Int32 tdx, tdy;
            XmlNodeList gridnode = FileUtil.xmlfile.SelectSingleNode("config/Grid").ChildNodes;
            Int32 gridnum = gridnode.Count;
            string tdname;
            string[] td;
            for (int p = 0; p < gridnum; p++)
            {
                tdname = gridnode[p].Name;
                td = tdname.Split(new string[] { "td", "-" }, StringSplitOptions.RemoveEmptyEntries);
                tdx = Convert.ToInt32(td[0]);
                tdy = Convert.ToInt32(td[1]);
                string strType = gridnode[p].InnerText.ToString().Trim();
                XmlAttribute xa = gridnode[p].Attributes["direction"];

                if (tdx >= ConstDefine.minX && tdx <= ConstDefine.maxX && tdy >=ConstDefine.minY&& tdy <=ConstDefine.maxY)
                {
                    mapnode[tdx, tdy].IsAbleCross = true;
                    mapnode[tdx, tdy].Type =MapNodeType.Road;
                }
                else
                {
                    mapnode[tdx, tdy].IsAbleCross = false;
                    mapnode[tdx, tdy].Type = MapNodeType.obstacle;
                }
                if (false)
                {
                    string dir = xa.InnerText.ToString().Trim();                   

                    if (dir == "0000")
                    {
                        mapnode[tdx, tdy].IsAbleCross = false;
                    }
                    mapnode[tdx, tdy].UpDifficulty    = (dir[0] == '1' ? MapNode.DEFAULT_DIFFICULTY : MapNode.UNABLE_PASS);
                    mapnode[tdx, tdy].DownDifficulty  = (dir[1] == '1' ? MapNode.DEFAULT_DIFFICULTY : MapNode.UNABLE_PASS);
                    mapnode[tdx, tdy].LeftDifficulty  = (dir[2] == '1' ? MapNode.DEFAULT_DIFFICULTY : MapNode.UNABLE_PASS);
                    mapnode[tdx, tdy].RightDifficulty = (dir[3] == '1' ? MapNode.DEFAULT_DIFFICULTY : MapNode.UNABLE_PASS);

                    //if ((tdy > 25 && tdy < 29) || (tdy > 44 && tdy < 49) || (tdy > 63 && tdy < 68))
                    //{
                    //    mapnode[tdx, tdy].UpDifficulty -= 1;
                    //    mapnode[tdx, tdy].DownDifficulty -= 1;
                    //}
                    //if (tdx > 31 && tdx < 35)
                    //{
                    //    mapnode[tdx, tdy].LeftDifficulty -= 2;
                    //    mapnode[tdx, tdy].RightDifficulty -= 2;
                    //}

                    if (tdy == 19 || tdy == 20 || tdy == 22 || tdy == 23 || tdy == 25)
                    {
                        mapnode[tdx, tdy].UpDifficulty += 1;
                        mapnode[tdx, tdy].DownDifficulty += 1;
                    }
                    if (tdy==68||tdy == 70 ||tdy == 71)
                    {
                        mapnode[tdx, tdy].UpDifficulty += 1;
                        mapnode[tdx, tdy].DownDifficulty += 1;
                    }


                    if (tdy == 16 || tdy == 17 || tdy == 73 || tdy == 74)
                    {
                        mapnode[tdx, tdy].UpDifficulty += 2;
                        mapnode[tdx, tdy].DownDifficulty += 2;
                    }

                }
                switch (strType)
                {

                    case "充电区": mapnode[tdx, tdy].Type = MapNodeType.chargingArea;
                        break;
                    case "道路": mapnode[tdx, tdy].Type = MapNodeType.Road;
                        break;
                    case "排队区入口": mapnode[tdx, tdy].Type = MapNodeType.queueEntra;
                        try
                        {
                            queueEntra.Add(new MyPoint(tdx, tdy));
                        }
                        catch (Exception ex)
                        {
                            MessageBox.Show("排队区入口点增加异常"+ex);
                            Logs.Error("排队区入口点增加异常"+ex);
                        }
                        break;
                    case "扫描仪": mapnode[tdx, tdy].Type = MapNodeType.scanner;
                        try
                        {
                            scanner.Add(new MyPoint(tdx, tdy));
                        }
                        catch (Exception ex)
                        {
                            MessageBox.Show("地图文件中有相同的扫描仪点"+ex);
                            Logs.Error("地图文件中有相同的扫描仪点"+ex);
                        }
                        break;
                    case "排队区": mapnode[tdx, tdy].Type = MapNodeType.queuingArea;
                        try
                        {
                            queuingArea.Add(new MyPoint(tdx, tdy));
                        }
                        catch (Exception ex)
                        {
                            MessageBox.Show("排队区点增加异常"+ex);
                            Logs.Error("排队区点增加异常"+ex);
                        }                        
                        break;
                    case "休息区": mapnode[tdx, tdy].Type = MapNodeType.restArea;
                        try
                        {
                            restArea.Add(new MyPoint(tdx, tdy));
                        }
                        catch (Exception ex)
                        {
                            MessageBox.Show("休息区点增加异常" + ex);
                            Logs.Error("休息区点增加异常" + ex);
                        }  
                        break;
                    case "工位": mapnode[tdx, tdy].Type = MapNodeType.platform;
                        break;
                    case "障碍区": mapnode[tdx, tdy].Type = MapNodeType.obstacle;
                        break;
                    case "投放口": mapnode[tdx, tdy].Type = MapNodeType.deliveryPort;
                        break;
                    case "禁入点": mapnode[tdx, tdy].Type = MapNodeType.forbidNode;
                        break;
                    default:
                        MessageBox.Show("未知的节点类型:" + strType);
                        Logs.Fatal("未知的节点类型:" + strType);
                        break;
                }


            }


        }
       public  MyPoint CalculateScannerPoint(MyPoint quePoint)
        {
            float minDis = float.MaxValue;
            MyPoint point = null;
            for (int i = 0; i < scanner.Count; i++)
            {
                float temp = SquareDistance(quePoint, scanner[i]);
                if (temp < minDis)
                {
                    minDis = temp;
                    point = scanner[i];

                }
            }
            return point;

        }
        float SquareDistance(MyPoint point1, MyPoint point2)
        {
            return (point1.X - point2.X) * (point1.X - point2.X) + (point1.Y - point2.Y) * (point1.Y - point2.Y);
        }
        //void CalcuScaDict()
        //{
        //    scannerDict = new Dictionary<MyPoint, MyPoint>();
        //    for (int i = 0; i < queueEntra.Count; i++)
        //    {
        //        try
        //        {
                    
        //            scannerDict.Add(queueEntra[i], CalculateScannerPoint(queueEntra[i]));
        //        }catch(Exception ex){
        //            Logs.Error("增加scannerDict键值对出错" + ex);
        //        }
        //    }

        //}
        //private Dictionary<MyPoint, MyPoint> scannerDict;

        //public MyPoint GetScannerPoint(int BeginX, int BeginY)
        //{            
            
        //    MyPoint key = new MyPoint(BeginX, BeginY);
        //    if (BeginX == 20 && BeginY == 83) { return key; }
        //    MyPoint value = null;
        //    try
        //    {
        //        value = scannerDict[key];
        //    }
        //    catch (Exception ex) { Logs.Error("给定的坐标没有在扫描仪字典中"); }
        //    return value;

        //}

        ///// <summary>
        ///// 把电子地图左侧的空白区域的图例放置好
        ///// </summary>
        ///// <param name="g"></param>
        //public void SetBlank(Graphics g)
        //{        
        // //道路、中间隔带、传送带、投放处、充电站、障碍物、扫描仪
        //    g.DrawImage(img_Road, ConstDefine.BLANK_X, ConstDefine.BLANK_Y);
        //    g.DrawImage(img_Mid, ConstDefine.BLANK_X, ConstDefine.BLANK_Y + ConstDefine.SPACE);
        //    g.DrawImage(img_Belt, ConstDefine.BLANK_X, ConstDefine.BLANK_Y + 2 * ConstDefine.SPACE);
        //    g.DrawImage(img_Destination, ConstDefine.BLANK_X, ConstDefine.BLANK_Y + 3 * ConstDefine.SPACE);
        //    g.DrawImage(img_ChargeStation, ConstDefine.BLANK_X, ConstDefine.BLANK_Y + 4 * ConstDefine.SPACE);
        //    g.DrawImage(img_Obstacle, ConstDefine.BLANK_X, ConstDefine.BLANK_Y + 5 * ConstDefine.SPACE);
        //    g.DrawImage(img_Scanner, ConstDefine.BLANK_X, ConstDefine.BLANK_Y + 6 * ConstDefine.SPACE);
        //}       
        public Boolean IsVehicleCanMove(Vehicle v, int x, int y)
        {
            Boolean temp = mapnode[x, y].IsAbleCross && (mapnode[x, y].NodeCanUsed == -1 || mapnode[x, y].NodeCanUsed == v.Id);
            return temp;
        }
        public Boolean IsNodeAvailable(int x, int y)
        {
            Boolean temp = mapnode[x, y].IsAbleCross && (mapnode[x, y].NodeCanUsed == -1);
            return temp;
        }
        public  bool IsLegalLocation(int X,int Y)
        {
            if (X < 0||Y < 0||X > HeightNum - 1||Y > WidthNum - 1)
            {
                return false;
            }
            return true;
        }

    }
}
