using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0
{
    class SearchData
    {
        private Object sdLock = new Object();
        private SendData _sendData;
        private bool _isReSearch;
        public SendData Data
        {
            get
            {
                return _sendData;
            }
            set
            {
                this._sendData = value;
            }
        }
        public bool IsReSearch
        {
            get
            {
                return _isReSearch;
            }
            set
            {
                this._isReSearch = value;
            }
        }
        public SearchData(SendData sendData, bool isReSearch)
        {
            this._sendData = sendData;
            this._isReSearch = isReSearch;
        }
    }
}
