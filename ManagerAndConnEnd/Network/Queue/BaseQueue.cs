using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0
{
    /// <summary>
    /// 线性安全的队列
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public class BaseQueue<T>where T:class
    {
         Queue<T> myQueue = new Queue<T>();

        //加一个锁
        private Object myQueueLock = new Object();

        //判断队列里是否有数据
        public bool IsHasData()
        {
            bool ret = false;
            lock (this.myQueueLock)
            {
                ret = (this.myQueue != null && this.myQueue.Count > 0);
            }
            return ret;
        }
        public bool Clear()
        {
            bool ret = false;
            lock (this.myQueueLock)
            {
                if (this.myQueue != null && this.myQueue.Count > 0)
                {
                    this.myQueue.Clear();
                }
                ret = true;
            }
            return ret;
        }

        //判断队列里是否有数据
        public int Count()
        {
            int ret = 0;
            lock (this.myQueueLock)
            {
                ret = this.myQueue.Count;
            }
            return ret;
        }

        //入队操作，增加数据
        public void Enqueue(T obj)
        {

            lock (this.myQueueLock)
            {
                this.myQueue.Enqueue(obj);
            }
        }
        public T Peek()
        {
            T obj = default(T);
            lock (this.myQueueLock)
            {
                bool has = (this.myQueue != null && this.myQueue.Count > 0);
                if (has)
                {
                    obj = this.myQueue.Peek() as T;
                }

            }
            return obj;
        }

        //出队操作，获得数据
        public T Dequeue()
        {
            T obj = default(T);
            lock (this.myQueueLock)
            {
                bool has = (this.myQueue != null && this.myQueue.Count > 0);
                if (has)
                {
                    obj = this.myQueue.Dequeue() as T;
                }

            }
            return obj;
        }

    }
}
