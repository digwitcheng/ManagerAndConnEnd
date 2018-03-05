using AGV_V1._0.Event;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace AGV_V1._0.Network.ThreadCode
{
    /*
     * 使用7*7的模板，检测模板内的小车数量，如果超过阈值就判定为拥堵
     * 把模板内的节点的拥堵值都增加     
     */
    class CheckCongestionThread:BaseThread
    {
        private const int MODEL_LEN = 7;
        private const int THRESHOLD = 26;//阈值
        private static CheckCongestionThread instance;
        public static CheckCongestionThread Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = new CheckCongestionThread();
                }
                return instance;
            }
        }
        protected override string ThreadName()
        {
            return "CheckCongestionThread";
        }
        protected override void Run()
        {
            try
            {
                TraCongestion2Zero();
                CheckTraCongestion();
                Thread.Sleep(AGV_V1._0.Util.ConstDefine.CHECK_CONGESTION);
            }
            catch (Exception ex)
            {
                OnShowMessage(this, new MessageEventArgs(ex.Message));
            }
        }

        private void CheckTraCongestion()
        {
            for (int i = MODEL_LEN/2; i < ElecMap.Instance.mapnode.GetLength(0)-MODEL_LEN/2; i+=2)
            {
                for (int j = MODEL_LEN/2; j < ElecMap.Instance.mapnode.GetLength(1)-MODEL_LEN/2; j+=2)
                {
                    int count = 0;
                    for (int p = i-MODEL_LEN/2; p < i+MODEL_LEN/2; p++)
                    {
                        for (int q = j-MODEL_LEN/2; q < j+MODEL_LEN/2; q++)
                        {
                            if (ElecMap.Instance.mapnode[p, q].NodeCanUsed > -1)//当前节点有车
                            {
                                count++;
                            }
                        }
                    }
                    if (count >= THRESHOLD)//当前模板超过thresold个节点上有车，把当前模板的节点的拥挤度都增加
                    {
                        for (int p = i - MODEL_LEN / 2; p < i + MODEL_LEN / 2; p++)
                        {
                            for (int q = j - MODEL_LEN / 2; q < j + MODEL_LEN / 2; q++)
                            {
                                ElecMap.Instance.mapnode[p, q].TraCongesIntensity++;
                            }
                        }
                    }
                }
            }
        }

        private void TraCongestion2Zero()
        {
            for (int i = 0; i < ElecMap.Instance.mapnode.GetLength(0); i++)
            {
                for (int j = 0; j < ElecMap.Instance.mapnode.GetLength(1); j++)
                {
                    ElecMap.Instance.mapnode[i, j].TraCongesIntensity = 0;
                }
            }
        }
    }
}
