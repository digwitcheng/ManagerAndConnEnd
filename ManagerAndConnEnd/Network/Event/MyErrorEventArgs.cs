using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Event
{
    public class MyErrorEventArgs : EventArgs
    {
        public string Message { get; set; }
        public Exception Exception { get; set; }
        public MyErrorEventArgs(string message)
            : base()
        {
            this.Message = message;
        }
        public MyErrorEventArgs(string message, Exception exception)
            : base()
        {
            this.Message = message;
            this.Exception = exception;
        }
    }
}
