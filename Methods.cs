using System;
using System.Buffers;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace win7
{
    public static class Methods
    {
        private static readonly SpanAction<char, IntPtr> _getAsciiStringNonNullCharacters = GetAsciiStringNonNullCharacters;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool AllBytesInUInt32AreAscii(uint value)
        {
            return ((value & 0x80808080u) == 0);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining | MethodImplOptions.AggressiveOptimization)]
        private static bool WidenFourAsciiBytesToUtf16AndCompareToChars(ref char charStart, uint value)
        {
            if (!AllBytesInUInt32AreAscii(value))
            {
                return false;
            }

            // BMI2 could be used, but this variant is faster on both Intel and AMD.
            if (Sse2.X64.IsSupported)
            {
                Vector128<byte> vecNarrow = Sse2.ConvertScalarToVector128UInt32(value).AsByte();
                Vector128<ulong> vecWide = Sse2.UnpackLow(vecNarrow, Vector128<byte>.Zero).AsUInt64();
                return Unsafe.ReadUnaligned<ulong>(ref Unsafe.As<char, byte>(ref charStart)) ==
                    Sse2.X64.ConvertToUInt64(vecWide);
            }
            else
            {
                if (BitConverter.IsLittleEndian)
                {
                    return charStart == (char)(byte)value &&
                        Unsafe.Add(ref charStart, 1) == (char)(byte)(value >> 8) &&
                        Unsafe.Add(ref charStart, 2) == (char)(byte)(value >> 16) &&
                        Unsafe.Add(ref charStart, 3) == (char)(value >> 24);
                }
                else
                {
                    return Unsafe.Add(ref charStart, 3) == (char)(byte)value &&
                        Unsafe.Add(ref charStart, 2) == (char)(byte)(value >> 8) &&
                        Unsafe.Add(ref charStart, 1) == (char)(byte)(value >> 16) &&
                        charStart == (char)(value >> 24);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool AllBytesInUInt16AreAscii(ushort value)
        {
            return ((value & 0x8080u) == 0);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining | MethodImplOptions.AggressiveOptimization)]
        private static bool WidenTwoAsciiBytesToUtf16AndCompareToChars(ref char charStart, ushort value)
        {
            if (!AllBytesInUInt16AreAscii(value))
            {
                return false;
            }

            // BMI2 could be used, but this variant is faster on both Intel and AMD.
            if (Sse2.IsSupported)
            {
                Vector128<byte> vecNarrow = Sse2.ConvertScalarToVector128UInt32(value).AsByte();
                Vector128<uint> vecWide = Sse2.UnpackLow(vecNarrow, Vector128<byte>.Zero).AsUInt32();
                return Unsafe.ReadUnaligned<uint>(ref Unsafe.As<char, byte>(ref charStart)) ==
                    Sse2.ConvertToUInt32(vecWide);
            }
            else
            {
                if (BitConverter.IsLittleEndian)
                {
                    return charStart == (char)(byte)value &&
                        Unsafe.Add(ref charStart, 1) == (char)(byte)(value >> 8);
                }
                else
                {
                    return Unsafe.Add(ref charStart, 1) == (char)(byte)value &&
                        charStart == (char)(byte)(value >> 8);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)] // Needs a push
        private static bool CheckBytesInAsciiRange(Vector<sbyte> check)
        {
            // Vectorized byte range check, signed byte > 0 for 1-127
            return Vector.GreaterThanAll(check, Vector<sbyte>.Zero);
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        public static bool BytesOrdinalEqualsStringAndAscii(string previousValue, ReadOnlySpan<byte> newValue, bool log)
        {
            // previousValue is a previously materialized string which *must* have already passed validation.
            //Debug.Assert(IsValidHeaderString(previousValue));

            // Ascii bytes => Utf-16 chars will be the same length.
            // The caller should have already compared lengths before calling this method.
            // However; let's double check, and early exit if they are not the same length.
            if (previousValue.Length != newValue.Length)
            {
                // Lengths don't match, so there cannot be an exact ascii conversion between the two.
                goto NotEqual;
            }

            // Use IntPtr values rather than int, to avoid unnecessary 32 -> 64 movs on 64-bit.
            // Unfortunately this means we also need to cast to byte* for comparisons as IntPtr doesn't
            // support operator comparisons (e.g. <=, >, etc).
            //
            // Note: Pointer comparison is unsigned, so we use the compare pattern (offset + length <= count)
            // rather than (offset <= count - length) which we'd do with signed comparison to avoid overflow.
            // This isn't problematic as we know the maximum length is max string length (from test above)
            // which is a signed value so half the size of the unsigned pointer value so we can safely add
            // a Vector<byte>.Count to it without overflowing.
            var count = (nint)newValue.Length;
            var offset = (nint)0;

            // Get references to the first byte in the span, and the first char in the string.
            ref var bytes = ref MemoryMarshal.GetReference(newValue);
            ref var str = ref MemoryMarshal.GetReference(previousValue.AsSpan());

            do
            {
                // If Vector not-accelerated or remaining less than vector size
                if (!Vector.IsHardwareAccelerated || (offset + Vector<byte>.Count) > count)
                {
                    if (IntPtr.Size == 8) // Use Intrinsic switch for branch elimination
                    {
                        // 64-bit: Loop longs by default
                        while ((offset + sizeof(long)) <= count)
                        {
                            if (!WidenFourAsciiBytesToUtf16AndCompareToChars(
                                    ref Unsafe.Add(ref str, offset),
                                    Unsafe.ReadUnaligned<uint>(ref Unsafe.Add(ref bytes, offset))) ||
                                !WidenFourAsciiBytesToUtf16AndCompareToChars(
                                    ref Unsafe.Add(ref str, offset + 4),
                                    Unsafe.ReadUnaligned<uint>(ref Unsafe.Add(ref bytes, offset + 4))))
                            {
                                if (log)
                                {
                                    Console.WriteLine("1");
                                }
                                goto NotEqual;
                            }

                            offset += sizeof(long);
                        }
                        if ((offset + sizeof(int)) <= count)
                        {
                            if (!WidenFourAsciiBytesToUtf16AndCompareToChars(
                                ref Unsafe.Add(ref str, offset),
                                Unsafe.ReadUnaligned<uint>(ref Unsafe.Add(ref bytes, offset))))
                            {
                                if (log)
                                {
                                    Console.WriteLine("2");
                                }
                                goto NotEqual;
                            }

                            offset += sizeof(int);
                        }
                    }
                    else
                    {
                        // 32-bit: Loop ints by default
                        while ((offset + sizeof(int)) <= count)
                        {
                            if (!WidenFourAsciiBytesToUtf16AndCompareToChars(
                                ref Unsafe.Add(ref str, offset),
                                Unsafe.ReadUnaligned<uint>(ref Unsafe.Add(ref bytes, offset))))
                            {
                                if (log)
                                {
                                    Console.WriteLine("3");
                                }
                                goto NotEqual;
                            }

                            offset += sizeof(int);
                        }
                    }
                    if ((offset + sizeof(short)) <= count)
                    {
                        if (!WidenTwoAsciiBytesToUtf16AndCompareToChars(
                            ref Unsafe.Add(ref str, offset),
                            Unsafe.ReadUnaligned<ushort>(ref Unsafe.Add(ref bytes, offset))))
                        {
                            if (log)
                            {
                                Console.WriteLine("4");
                            }
                            goto NotEqual;
                        }

                        offset += sizeof(short);
                    }
                    if (offset < count)
                    {
                        var ch = (char)Unsafe.Add(ref bytes, offset);
                        if (((ch & 0x80) != 0) || Unsafe.Add(ref str, offset) != ch)
                        {
                            if (log)
                            {
                                Console.WriteLine("5");
                            }
                            goto NotEqual;
                        }
                    }

                    // End of input reached, there are no inequalities via widening; so the input bytes are both ascii
                    // and a match to the string if it was converted via Encoding.ASCII.GetString(...)
                    return true;
                }

                // Create a comparision vector for all bits being equal
                var AllTrue = new Vector<short>(-1);
                // do/while as entry condition already checked, remaining length must be Vector<byte>.Count or larger.
                do
                {
                    // Read a Vector length from the input as bytes
                    var vector = Unsafe.ReadUnaligned<Vector<sbyte>>(ref Unsafe.Add(ref bytes, offset));
                    if (!CheckBytesInAsciiRange(vector))
                    {
                        if (log)
                        {
                            Console.WriteLine("6");
                        }
                        goto NotEqual;
                    }
                    // Widen the bytes directly to chars (ushort) as if they were ascii.
                    // As widening doubles the size we get two vectors back.
                    Vector.Widen(vector, out var vector0, out var vector1);
                    // Read two char vectors from the string to perform the match.
                    var compare0 = Unsafe.ReadUnaligned<Vector<short>>(ref Unsafe.As<char, byte>(ref Unsafe.Add(ref str, offset)));
                    var compare1 = Unsafe.ReadUnaligned<Vector<short>>(ref Unsafe.As<char, byte>(ref Unsafe.Add(ref str, offset + Vector<ushort>.Count)));

                    // If the string is not ascii, then the widened bytes cannot match
                    // as each widened byte element as chars will be in the range 0-255
                    // so cannot match any higher unicode values.

                    // Compare to our all bits true comparision vector
                    if (!AllTrue.Equals(
                        // BitwiseAnd the two equals together
                        Vector.BitwiseAnd(
                            // Check equality for the two widened vectors
                            Vector.Equals(compare0, vector0),
                            Vector.Equals(compare1, vector1))))
                    {
                        if (log)
                        {
                            Console.WriteLine("7");
                            Console.WriteLine($"offset: {offset}, vector: {vector}, compare0: {compare0}, vector0: {vector0}, compare1: {compare1}, vector1: {vector1}");
                        }
                        goto NotEqual;
                    }

                    offset += Vector<byte>.Count;
                } while ((offset + Vector<byte>.Count) <= count);

                // Vector path done, loop back to do non-Vector
                // If is a exact multiple of vector size, bail now
            } while (offset < count);

            // If we get here (input is exactly a multiple of Vector length) then there are no inequalities via widening;
            // so the input bytes are both ascii and a match to the string if it was converted via Encoding.ASCII.GetString(...)
            return true;
        NotEqual:
            return false;
        }

        private static unsafe void GetAsciiStringNonNullCharacters(Span<char> buffer, IntPtr state)
        {
            fixed (char* output = &MemoryMarshal.GetReference(buffer))
            {
                // StringUtilities.TryGetAsciiString returns null if there are any null (0 byte) characters
                // in the string
                if (!TryGetAsciiString((byte*)state.ToPointer(), output, buffer.Length))
                {
                    throw new InvalidOperationException();
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe string GetAsciiStringNonNullCharacters(ReadOnlySpan<byte> span)
        {
            if (span.IsEmpty)
            {
                return string.Empty;
            }

            fixed (byte* source = &MemoryMarshal.GetReference(span))
            {
                return string.Create(span.Length, new IntPtr(source), _getAsciiStringNonNullCharacters);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        public static unsafe bool TryGetAsciiString(byte* input, char* output, int count)
        {
            Debug.Assert(input != null);
            Debug.Assert(output != null);

            var end = input + count;

            Debug.Assert((long)end >= Vector256<sbyte>.Count);

            // PERF: so the JIT can reuse the zero from a register
            Vector128<sbyte> zero = Vector128<sbyte>.Zero;

            if (Sse2.IsSupported)
            {
                if (Avx2.IsSupported && input <= end - Vector256<sbyte>.Count)
                {
                    Vector256<sbyte> avxZero = Vector256<sbyte>.Zero;

                    do
                    {
                        var vector = Avx.LoadVector256(input).AsSByte();
                        if (!CheckBytesInAsciiRange(vector, avxZero))
                        {
                            return false;
                        }

                        var tmp0 = Avx2.UnpackLow(vector, avxZero);
                        var tmp1 = Avx2.UnpackHigh(vector, avxZero);

                        // Bring into the right order
                        var out0 = Avx2.Permute2x128(tmp0, tmp1, 0x20);
                        var out1 = Avx2.Permute2x128(tmp0, tmp1, 0x31);

                        Avx.Store((ushort*)output, out0.AsUInt16());
                        Avx.Store((ushort*)output + Vector256<ushort>.Count, out1.AsUInt16());

                        input += Vector256<sbyte>.Count;
                        output += Vector256<sbyte>.Count;
                    } while (input <= end - Vector256<sbyte>.Count);

                    if (input == end)
                    {
                        return true;
                    }
                }

                if (input <= end - Vector128<sbyte>.Count)
                {
                    do
                    {
                        var vector = Sse2.LoadVector128(input).AsSByte();
                        if (!CheckBytesInAsciiRange(vector, zero))
                        {
                            return false;
                        }

                        var c0 = Sse2.UnpackLow(vector, zero).AsUInt16();
                        var c1 = Sse2.UnpackHigh(vector, zero).AsUInt16();

                        Sse2.Store((ushort*)output, c0);
                        Sse2.Store((ushort*)output + Vector128<ushort>.Count, c1);

                        input += Vector128<sbyte>.Count;
                        output += Vector128<sbyte>.Count;
                    } while (input <= end - Vector128<sbyte>.Count);

                    if (input == end)
                    {
                        return true;
                    }
                }
            }
            else if (Vector.IsHardwareAccelerated)
            {
                while (input <= end - Vector<sbyte>.Count)
                {
                    var vector = Unsafe.AsRef<Vector<sbyte>>(input);
                    if (!CheckBytesInAsciiRange(vector))
                    {
                        return false;
                    }

                    Vector.Widen(
                        vector,
                        out Unsafe.AsRef<Vector<short>>(output),
                        out Unsafe.AsRef<Vector<short>>(output + Vector<short>.Count));

                    input += Vector<sbyte>.Count;
                    output += Vector<sbyte>.Count;
                }

                if (input == end)
                {
                    return true;
                }
            }

            if (Environment.Is64BitProcess) // Use Intrinsic switch for branch elimination
            {
                // 64-bit: Loop longs by default
                while (input <= end - sizeof(long))
                {
                    var value = *(long*)input;
                    if (!CheckBytesInAsciiRange(value))
                    {
                        return false;
                    }

                    // BMI2 could be used, but this variant is faster on both Intel and AMD.
                    if (Sse2.X64.IsSupported)
                    {
                        Vector128<sbyte> vecNarrow = Sse2.X64.ConvertScalarToVector128Int64(value).AsSByte();
                        Vector128<ulong> vecWide = Sse2.UnpackLow(vecNarrow, zero).AsUInt64();
                        Sse2.Store((ulong*)output, vecWide);
                    }
                    else
                    {
                        output[0] = (char)input[0];
                        output[1] = (char)input[1];
                        output[2] = (char)input[2];
                        output[3] = (char)input[3];
                        output[4] = (char)input[4];
                        output[5] = (char)input[5];
                        output[6] = (char)input[6];
                        output[7] = (char)input[7];
                    }

                    input += sizeof(long);
                    output += sizeof(long);
                }

                if (input <= end - sizeof(int))
                {
                    var value = *(int*)input;
                    if (!CheckBytesInAsciiRange(value))
                    {
                        return false;
                    }

                    WidenFourAsciiBytesToUtf16AndWriteToBuffer(output, input, value, zero);

                    input += sizeof(int);
                    output += sizeof(int);
                }
            }
            else
            {
                // 32-bit: Loop ints by default
                while (input <= end - sizeof(int))
                {
                    var value = *(int*)input;
                    if (!CheckBytesInAsciiRange(value))
                    {
                        return false;
                    }

                    WidenFourAsciiBytesToUtf16AndWriteToBuffer(output, input, value, zero);

                    input += sizeof(int);
                    output += sizeof(int);
                }
            }

            if (input <= end - sizeof(short))
            {
                if (!CheckBytesInAsciiRange(((short*)input)[0]))
                {
                    return false;
                }

                output[0] = (char)input[0];
                output[1] = (char)input[1];

                input += sizeof(short);
                output += sizeof(short);
            }

            if (input < end)
            {
                if (!CheckBytesInAsciiRange(((sbyte*)input)[0]))
                {
                    return false;
                }
                output[0] = (char)input[0];
            }

            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static unsafe void WidenFourAsciiBytesToUtf16AndWriteToBuffer(char* output, byte* input, int value, Vector128<sbyte> zero)
        {
            // BMI2 could be used, but this variant is faster on both Intel and AMD.
            if (Sse2.X64.IsSupported)
            {
                Vector128<sbyte> vecNarrow = Sse2.ConvertScalarToVector128Int32(value).AsSByte();
                Vector128<ulong> vecWide = Sse2.UnpackLow(vecNarrow, zero).AsUInt64();
                Unsafe.WriteUnaligned(output, Sse2.X64.ConvertToUInt64(vecWide));
            }
            else
            {
                output[0] = (char)input[0];
                output[1] = (char)input[1];
                output[2] = (char)input[2];
                output[3] = (char)input[3];
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool CheckBytesInAsciiRange(int check)
        {
            const int HighBits = unchecked((int)0x80808080);
            return (((check - 0x01010101) | check) & HighBits) == 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool CheckBytesInAsciiRange(short check)
        {
            const short HighBits = unchecked((short)0x8080);
            return (((short)(check - 0x0101) | check) & HighBits) == 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)] // Needs a push
        private static bool CheckBytesInAsciiRange(long check)
        {
            const long HighBits = unchecked((long)0x8080808080808080L);
            return (((check - 0x0101010101010101L) | check) & HighBits) == 0;
        }

        private static bool CheckBytesInAsciiRange(sbyte check)
            => check > 0;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool CheckBytesInAsciiRange(Vector128<sbyte> check, Vector128<sbyte> zero)
        {
            Debug.Assert(Sse2.IsSupported);

            var mask = Sse2.CompareGreaterThan(check, zero);
            return Sse2.MoveMask(mask) == 0xFFFF;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool CheckBytesInAsciiRange(Vector256<sbyte> check, Vector256<sbyte> zero)
        {
            Debug.Assert(Avx2.IsSupported);

            var mask = Avx2.CompareGreaterThan(check, zero);
            return (uint)Avx2.MoveMask(mask) == 0xFFFF_FFFF;
        }
    }
}
