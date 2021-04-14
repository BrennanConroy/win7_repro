using System;
using System.Buffers;
using System.Linq;

namespace win7
{
    class Program
    {
        static void Main(string[] args)
        {
            var byteRange = Enumerable.Range(1, 127).Select(x => (byte)x);
            var byteArray = byteRange
                .Concat(byteRange)
                .Concat(byteRange)
                .Concat(byteRange)
                .Concat(byteRange)
                .Concat(byteRange)
                .ToArray();
            var span = new Span<byte>(byteArray);

            Console.WriteLine("Starting test");

            var iterations = 1000;

            for (var iter = 0; iter < iterations; iter++)
            {
                for (var i = 0; i <= byteArray.Length; i++)
                {
                    // Test all the lengths to hit all the different length paths e.g. Vector, long, short, char
                    Test(span.Slice(i));
                }
            }

            Console.WriteLine("Finished successfully");

            static void Test(Span<byte> asciiBytes)
            {
                var s = Methods.GetAsciiStringNonNullCharacters(asciiBytes);

                // Should start as equal
                if (!Methods.BytesOrdinalEqualsStringAndAscii(s, asciiBytes, log: true))
                {
                    throw new Exception("Failed first equals check");
                }
                for (var i = 0; i < asciiBytes.Length; i++)
                {
                    var b = asciiBytes[i];
                    // Change one byte, ensure is not equal
                    asciiBytes[i] = (byte)(b + 1);
                    if (Methods.BytesOrdinalEqualsStringAndAscii(s, asciiBytes, log: false))
                    {
                        throw new Exception("Failed not equals check");
                    }

                    // Change byte back for next iteration, ensure is equal again
                    asciiBytes[i] = b;
                    if (!Methods.BytesOrdinalEqualsStringAndAscii(s, asciiBytes, log: true))
                    {
                        throw new Exception("Failed second equals check");
                    }
                }
            }
        }
    }
}
