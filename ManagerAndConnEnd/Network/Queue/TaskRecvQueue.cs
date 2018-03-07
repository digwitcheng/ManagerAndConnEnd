using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Queue
{
    class TaskRecvQueue : BaseQueue<string>
    {
        private static TaskRecvQueue instance;
        public static TaskRecvQueue Instance
        {
            get
            {
                if (null == instance)
                {
                    instance = new TaskRecvQueue();
                }
                return instance;
            }
        }
    }
}
