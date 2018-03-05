using AGV_V1._0.Network.ThreadCode;
using AGV_V1._0.Queue;
using AGV_V1._0.Network;
using AGV_V1._0.NLog;
using AGV_V1._0.Util;
using DataBase;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Data;
using System.Data.SqlClient;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using AGVSocket.Network;

namespace AGV_V1._0.DataBase
{
    class UpdataSqlThread : BaseThread
    {
        private static SqlConnection sqlConn;
        private const int MAX_TRY_CONN_COUNT = 100;
        private int connCount = 1;
        private static UpdataSqlThread instance;
        public static UpdataSqlThread Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = new UpdataSqlThread();
                }
                return instance;
            }
        }
        private UpdataSqlThread()
        {
            // Task.Factory.StartNew(() => TryConnect2DataBase());//启动线程            
            TryConnect2DataBase();
        }
        void TryConnect2DataBase()
        {
            while (sqlConn == null && connCount < MAX_TRY_CONN_COUNT)
            {
                sqlConn = SqlHelper.GetSqlConnection();
                Thread.Sleep(connCount * 100);
                connCount++;
            }
            if (connCount >= MAX_TRY_CONN_COUNT)
            {
                Logs.Warn("未能连上数据库");
            }
        }
        public void GetElecMapInfo()
        {
            if (sqlConn != null && sqlConn.State == ConnectionState.Open)
            {
                DataTable data = SqlHelper.GetDataTable(sqlConn, "select * from ElecMap");
                if (data != null)
                {
                    Console.WriteLine(data.Rows[0]["Info"]);
                }
            }
            else
            {
                Console.WriteLine("数据库未连接");
            }
        }
        public void Update(string CmdText)
        {
            SqlHelper.ExecNonQuery(sqlConn, CmdText);
        }

        protected override string ThreadName()
        {
            return "UpdataSqlThread";
        }
        protected override void Run()
        {
            try
            {
                if (sqlConn != null && sqlConn.State == ConnectionState.Open)
                {
                    //if (CmdTxtQueue.Instance.IsHasData())
                    //{
                    //    string cmdStr = CmdTxtQueue.Instance.Dequeue();
                    //    SqlHelper.ExecNonQuery(sqlConn, cmdStr);
                    //}

                    ConcurrentDictionary<ushort, AgvInfo> agvInfo = AgvServerManager.Instance.GetAgvInfo;
                    foreach (var key in agvInfo.Keys)
                    {
                        int id = key;
                        AgvInfo info = null;
                        agvInfo.TryGetValue(key, out info);
                        if (info != null)
                        {
                            string str = string.Format("update Vehicle set CurX={0} ,CurY={1}, DesX={2}, DesY={3} where Id={4}",
                                info.CurLocation.CurNode.X, info.CurLocation.CurNode.Y,
                                info.CurLocation.DesNode.X, info.CurLocation.DesNode.Y,
                                id);
                            SqlHelper.ExecNonQuery(sqlConn,str);
                        }
                    }   
                }
                else
                {
                    Console.WriteLine("数据库未连接");
                }
                Thread.Sleep(ConstDefine.UPDATA_SQL_TIME);
            }
            catch (Exception ex)
            {
                Logs.Error("更新小车信息到数据库异常，退出UpdatasqlThread线程，异常：" + ex.ToString());
            }
        }
    }
}
