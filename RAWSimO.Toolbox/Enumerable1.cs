using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RAWSimO.Toolbox
{
    public static class Enumerable1
    {
        public static IEnumerable<IEnumerable<TSource>> Combo<TSource>(this IEnumerable<IEnumerable<TSource>> source, IEnumerable<TSource> second)
        {
            foreach (var i in second)
            {
                foreach (var j in source)
                {
                    List<TSource> list = new List<TSource>();
                    list.Add(i);
                    yield return j.Concat(list);
                }
            }
        }

        public static IEnumerable<IEnumerable<TSource>> StartCombo<TSource>(this IEnumerable<TSource> first)
        {
            foreach (var i in first)
            {
                List<TSource> list = new List<TSource>();
                list.Add(i);
                yield return list;
            }
        }
    }
}
