using AGV_V1._0.Agv;
using AGV_V1._0.Util;
using Astar;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace AGV_V1._0
{
    class SearchManager
    {
        ElecMap Elc;
        AstarSearch astarSearch;
        //private readonly object searchLock = new object();

        private static SearchManager _instance;
        public static SearchManager Instance
        {
            get
            {
                if (_instance == null)
                {
                    _instance = new SearchManager();
                }
                return _instance;
            }
        }
        public SearchManager()
        {
            Elc = ElecMap.Instance;
            astarSearch = new AstarSearch();

        }
        private int ResearchCount
        {
            get;
            set;

        }
        public void ReSearchRoute(Vehicle v)
        {
            //lock (searchLock)
            //{
            ResearchCount++;
            if (v.Route == null || v.Route.Count <= v.VirtualTPtr)
            {
                return;
            }
            //if (ResearchCount > ConstDefine.RESEARCH_COUNT && v.BeginX == route[0].X && v.BeginY == route[0].Y)
            //{
            //    this.vehical_state = State.cannotToDestination;
            //    route = null;
            //    return;
            //}
            v.LockNode.Add(v.Route[v.VirtualTPtr]);
            //Elc.mapnode[route[Virtual_tPtr].X, route[Virtual_tPtr].Y].LockNode = v_num;
            for (int i = 1; i < ConstDefine.FORWORD_STEP - 1; i++)
            {
                if (v.TPtr + i < v.Route.Count - 1)
                {
                    Elc.mapnode[v.Route[v.TPtr + i].X, v.Route[v.TPtr + i].Y].NodeCanUsed = -1;
                }
            }
            //Elc.mapnode[Route[8].X, Route[8].Y].LockNode = v_num;
            v.BeginX = v.Route[v.TPtr].X;
            v.BeginY = v.Route[v.TPtr].Y;
            //Task.Factory.StartNew(() => SearchRoute(Elc), TaskCreationOptions.None);
            SearchRoute(v);
            //}
            // Elc.mapnode[Route[Virtual_tPtr].X, Route[Virtual_tPtr].Y].LockNode = -1;
        }
        public void SearchRoute(Vehicle v)
        {
            v.RouteIndex = 0;
            v.cost = 0;
            v.TPtr = 0;// tFram = 0;
            v.StopTime = ConstDefine.STOP_TIME;
            if (!checkXY(v))
            {
                v.CurState = State.cannotToDestination;
                MessageBox.Show("起点或终点超出地图界限");
                return;
            }
            ////AstarSearch astarSearch = new AstarSearch(Elc);
            List<MyPoint> scannerNode = new List<MyPoint>();
            if (!Elc.IsSpecialArea(v.BeginX, v.BeginY))
            {
                scannerNode = Elc.GetScanner();
            }
            List<MyPoint> routeList = astarSearch.Search(Elc,scannerNode, v.LockNode, v.Id, Elc.WidthNum, Elc.HeightNum, v.BeginX, v.BeginY, v.EndX, v.EndY, v.Dir);
            //this.Speed = 0;
            Elc.mapnode[v.BeginX, v.BeginY].NodeCanUsed = v.Id;
            // Elc.mapnode[startX, startY].NodeCanUsed = false;//搜索完,小车自己所在的地方被小车占用           
            if (routeList.Count<1)
            {
                // MessageBox.Show("没有搜索到路线:"+v_num);
                v.CurState = State.cannotToDestination;
                //v.LockNode.cl;
            }
            else
            {
                v.Route = routeList;
                if (Elc.IsQueueEntra(v.EndX, v.EndY))
                {

                    MyPoint nextEnd = ElecMap.Instance.CalculateScannerPoint(new MyPoint(v.EndX, v.EndY));
                    List<MyPoint> addRoute = astarSearch.Search(Elc,new List<MyPoint>(), v.LockNode, v.Id, Elc.WidthNum, Elc.HeightNum, v.EndX, v.EndY, nextEnd.X, nextEnd.Y, v.Dir);
                    if (addRoute != null && addRoute.Count > 1)
                    {
                        for (int i = 1; i < addRoute.Count; i++)
                        {
                            v.Route.Add(addRoute[i]);
                        }
                        //v.EndX = nextEnd.X;
                        //v.EndY = nextEnd.Y;
                        v.EndLoc = "ScanArea";
                    }
                }

            }
        }
        
        bool checkXY(Vehicle v)
        {
            if (v == null)
            {
                return false;
            }
            if (Elc.IsLegalLocation(v.BeginX, v.BeginY) && Elc.IsLegalLocation(v.EndX, v.EndY))
            {
                return true;
            }
            return false;
        }


    }
}
