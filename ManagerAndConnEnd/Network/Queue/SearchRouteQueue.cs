using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Queue
{
    class SearchRouteQueue : BaseQueue<SearchData>
    {
        private static SearchRouteQueue instance;
        public static SearchRouteQueue Instance
        {
            get
            {
                if (null == instance)
                {
                    instance = new SearchRouteQueue();
                }
                return instance;
            }
        }
    }
}
