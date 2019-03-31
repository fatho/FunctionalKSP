using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KerboTools
{
    class DisposeUtil
    {
        public static void DisposeFields(object instance)
        {
            Type type = instance.GetType();
            var fields = type.GetFields(System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Public);
            
            foreach (var field in fields)
            {
                object value = field.GetValue(instance);

                try
                {
                    TryDispose(value);
                }
                catch(Exception ex)
                {
                    Console.Error.WriteLine("Failed to dispose {0}: {1}", field.Name, ex);
                }
            }
        }

        private static void TryDispose(object value)
        {
            if (value is IDisposable disposable)
            {
                disposable.Dispose();
            }
            else if (value is Array arr)
            {
                foreach (var val in arr)
                {
                    TryDispose(val);
                }
            }
        }
    }
}
