using AGV_V1._0.Event;
using AGV_V1._0.Network.ThreadCode;
using AGV_V1._0.NLog;
using AGV_V1._0.Queue;
using AGV_V1._0.Util;
using CowboyTest.Server.APM;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace AGV_V1._0.ThreadCode
{
    class SearchRouteThread:BaseThread
    {
        private static SearchRouteThread instance;
        public static SearchRouteThread Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = new SearchRouteThread();
                }
                return instance;
            }
        }
        private SearchRouteThread()
        {
        }
        protected override string ThreadName()
        {
            return "SearchRoute";
        }
        protected override void Run()
        {
            if (SearchRouteQueue.Instance.IsHasData())
            {
                // Debug.WriteLine(SearchRouteQueue.Instance.GetMyQueueCount() + "");
                //搜索路径
                SearchData td = SearchRouteQueue.Instance.Dequeue();
                if (td != null)
                {
                    SearchRoute(td);
                }
            }
            Thread.Sleep(ConstDefine.SEARCH_TIME);
        }
        void SearchRoute(SearchData searchData)
        {
            Debug.Assert(searchData != null, "");
            if (searchData == null)
            {
                Logs.Warn("要搜索的任务为空");
                OnShowMessage(this, new MessageEventArgs("要搜索的任务为空"));
                return;
            }
            Vehicle[] vehicle = VehicleManager.Instance.GetVehicles();
            Debug.Assert(vehicle != null, "");
            if (vehicle == null)
            {
                Logs.Warn("小车还未初始化");
                OnShowMessage(this, new MessageEventArgs("小车还未初始化"));
                return;
            }
            int num = searchData.Data.Num;
            Debug.Assert(num < vehicle.Length && num > -1, "");
            if (num < 0 || num > vehicle.Length - 1)
            {
                Logs.Error("任务中小车编号超出小车实际数量");
                OnShowMessage(this, new MessageEventArgs("任务中小车编号超出小车实际数量"));
                return;
            }
            vehicle[num].BeginX = searchData.Data.BeginX;
            vehicle[num].BeginY = searchData.Data.BeginY;
            vehicle[num].EndX = searchData.Data.EndX;
            vehicle[num].EndY = searchData.Data.EndY; 
            vehicle[num].Arrive = searchData.Data.Arrive;
            vehicle[num].EndLoc = searchData.Data.EndLoc;
            vehicle[num].StartLoc = searchData.Data.StartLoc;
            vehicle[num].CurState = searchData.Data.State;
            if (searchData.IsReSearch)
            {
              // Task.Factory.StartNew(() => vehicle[num].SearchRoute(), TaskCreationOptions.LongRunning);
               // vehicle [num].ReSearchRoute();
                SearchManager.Instance.ReSearchRoute(vehicle[num]);
            }
            else
            {
               // Task.Factory.StartNew(() => vehicle[num].SearchRoute(), TaskCreationOptions.LongRunning);
               // vehicle[num].SearchRoute();
                SearchManager.Instance.SearchRoute(vehicle[num]);
            }

        }
    }
}
