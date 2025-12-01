#include "stdafx.h"
#include "PointFileReader.h"

/*
 * strtod.c --
 *
 *	Source code for the "strtod" library procedure.
 *
 * Copyright (c) 1988-1993 The Regents of the University of California.
 * Copyright (c) 1994 Sun Microsystems, Inc.
 *
 * Permission to use, copy, modify, and distribute this
 * software and its documentation for any purpose and without
 * fee is hereby granted, provided that the above copyright
 * notice appear in all copies.  The University of California
 * makes no representations about the suitability of this
 * software for any purpose.  It is provided "as is" without
 * express or implied warranty.
 *
 * RCS: @(#) $Id: strtod.c,v 1.3 2000/02/17 07:11:22 matz Exp $
 */

 //-==-==-===-=-=-=-==-=-=-=-=--=-==-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=
// constants for strtodd
static int maxExponent = 511;   /* Largest possible base 10 exponent.  Any
                                 * exponent larger than this will already
                                 * produce underflow or overflow, so there's
                                 * no need to worry about additional digits.
                                 */
static double powersOf10[] = {  /* Table giving binary powers of 10.  Entry */
        10.,                    /* is 10^2^i.  Used to convert decimal */
        100.,                   /* exponents into floating-point numbers. */
        1.0e4,
        1.0e8,
        1.0e16,
        1.0e32,
        1.0e64,
        1.0e128,
        1.0e256
};

static double strtodd(const char* string, char** endPtr)
{
    int sign, expSign = FALSE;
    double fraction, dblExp, * d;
    const char* p;
    int c;
    int exp = 0;
    int fracExp = 0;
    int mantSize;
    int decPt;
    const char* pExp;
    p = string;
    while (isspace(*p))
    {
        p += 1;
    }
    if (*p == '-')
    {
        sign = TRUE;
        p += 1;
    }
    else
    {
        if (*p == '+')
        {
            p += 1;
        }
        sign = FALSE;
    }
    decPt = -1;
    for (mantSize = 0;; mantSize += 1)
    {
        c = *p;
        if (!isdigit(c))
        {
            if ((c != '.') || (decPt >= 0))
            {
                break;
            }
            decPt = mantSize;
        }
        p += 1;
    }
    pExp = p;
    p -= mantSize;
    if (decPt < 0)
    {
        decPt = mantSize;
    }
    else
    {
        mantSize -= 1;
    }
    if (mantSize > 18)
    {
        fracExp = decPt - 18;
        mantSize = 18;
    }
    else
    {
        fracExp = decPt - mantSize;
    }
    if (mantSize == 0)
    {
        fraction = 0.0;
        p = string;
        goto done;
    }
    else
    {
        int frac1, frac2;
        frac1 = 0;
        for (; mantSize > 9; mantSize -= 1)
        {
            c = *p;
            p += 1;
            if (c == '.')
            {
                c = *p;
                p += 1;
            }
            frac1 = 10 * frac1 + (c - '0');
        }
        frac2 = 0;
        for (; mantSize > 0; mantSize -= 1)
        {
            c = *p;
            p += 1;
            if (c == '.')
            {
                c = *p;
                p += 1;
            }
            frac2 = 10 * frac2 + (c - '0');
        }
        fraction = (1.0e9 * frac1) + frac2;
    }

    p = pExp;
    if ((*p == 'E') || (*p == 'e'))
    {
        p += 1;
        if (*p == '-')
        {
            expSign = TRUE;
            p += 1;
        }
        else
        {
            if (*p == '+')
            {
                p += 1;
            }
            expSign = FALSE;
        }
        while (isdigit(*p))
        {
            exp = exp * 10 + (*p - '0');
            p += 1;
        }
    }
    if (expSign)
    {
        exp = fracExp - exp;
    }
    else
    {
        exp = fracExp + exp;
    }
    if (exp < 0)
    {
        expSign = TRUE;
        exp = -exp;
    }
    else
    {
        expSign = FALSE;
    }
    if (exp > maxExponent)
    {
        exp = maxExponent;
        errno = ERANGE;
    }
    dblExp = 1.0;
    for (d = powersOf10; exp != 0; exp >>= 1, d += 1)
    {
        if (exp & 01)
        {
            dblExp *= *d;
        }
    }
    if (expSign)
    {
        fraction /= dblExp;
    }
    else
    {
        fraction *= dblExp;
    }

done:
    if (endPtr != NULL)
    {
        *endPtr = (char*)p;
    }

    if (sign)
    {
        return -fraction;
    }
    return fraction;
}

//-==-==-===-=-=-=-==-=-=-=-=--=-==-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=
// utf8_to_wstr
static std::wstring utf8_to_wstr(const std::string_view str8) noexcept
{
    const int count = MultiByteToWideChar(CP_UTF8, 0, str8.data(), (int)str8.length(), NULL, 0);
    std::wstring wstr(count, 0);
    if (count > 0)
        MultiByteToWideChar(CP_UTF8, 0, str8.data(), (int)str8.length(), &wstr[0], count);
    return wstr;
}

//-==-==-===-=-=-=-==-=-=-=-=--=-==-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=
// FileHnd
class FileHnd
{
public:
    FileHnd(HANDLE h)
        :_h(h)
    {
    }

    ~FileHnd()
    {
        if (CloseHandle(_h) == FALSE)
            assert(0);
    }

    HANDLE hnd() const
    {
        return _h;
    }

private:
    HANDLE _h;
};

//-==-==-===-=-=-=-==-=-=-=-=--=-==-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=
// FileMap
class FileMap
{
public:
    FileMap(LPVOID fmap, size_t len)
        :_fmap((char*)fmap), _fend((char*)fmap + len), _len(len)
    {
    }

    ~FileMap()
    {
        if (UnmapViewOfFile(_fmap) == FALSE)
            assert(0);
    }

    char* beginf() const
    {
        return _fmap;
    }

    char* endf() const
    {
        return _fend;
    }

    size_t size() const
    {
        return _len;
    }

    std::string_view view() const
    {
        return std::string_view(_fmap, _len);
    }

private:
    char* _fmap = nullptr;
    char* _fend = nullptr;
    size_t _len = 0;
};

//-==-==-===-=-=-=-==-=-=-=-=--=-==-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=
// chunck the file into line-aligned segments
static std::vector<std::string_view> makefileSegments(const std::string_view fv)
{
    std::vector<std::string_view> segments;
    const size_t hw_threads = std::thread::hardware_concurrency() ? std::thread::hardware_concurrency() : 4;
    constexpr size_t min_chunk_size = 16 * 1024; // 16 KB, best?
    segments.reserve(hw_threads);

    if (fv.size() <= min_chunk_size || hw_threads <= 1)
    {
        segments.push_back(fv);
    }
    else
    {
        const size_t approx_chunk = fv.size() / hw_threads;
        const char* data = fv.data();
        const char* endp = data + fv.length();
        for (size_t i = 0; i < hw_threads; ++i)
        {
            const char* seg_begin = data + i * approx_chunk;
            const char* seg_end = (i + 1 == hw_threads) ? endp : (data + (i + 1) * approx_chunk);

            if (seg_begin != data && seg_begin < endp)
            {
                size_t remaining = static_cast<size_t>(endp - seg_begin);
                const void* p = memchr(seg_begin, '\n', remaining);
                if (p)
                    seg_begin = static_cast<const char*>(p) + 1;
                else
                    seg_begin = endp; // nothing left
            }
            if (seg_end < endp)
            {
                size_t remaining = static_cast<size_t>(endp - seg_end);
                const void* p = memchr(seg_end, '\n', remaining);
                if (p)
                    seg_end = static_cast<const char*>(p); // will include up to the '\n' in this segment parsing
                else
                    seg_end = endp;
            }
            if (seg_begin < seg_end)
            {
                segments.emplace_back(seg_begin, static_cast<size_t>(seg_end - seg_begin));
            }
        }
    }
    return segments;
}


static void parse_pnezd_range(std::string_view range, char dlm, PNEZDArray& out)
{
    size_t pos = 0, len = range.size();
    while (pos < len)
    {
        PNEZD pnezd{};

        // p
        size_t p = range.find(dlm, pos);
        if (p == std::string_view::npos) [[unlikely]]
            break;
        const auto id_str = range.substr(pos, p - pos);
        if (std::isdigit(id_str[0]))
            pnezd._id = static_cast<size_t>(strtoll(id_str.data(), NULL, 10));
        else
            pnezd._id = utf8_to_wstr(id_str);
        pos = p + 1;

        // n
        size_t n = range.find(dlm, pos);
        if (n == std::string_view::npos) [[unlikely]]
            break;
        pnezd.point[1] = strtodd(range.data() + pos, NULL);
        pos = n + 1;

        // e
        size_t e = range.find(dlm, pos);
        if (e == std::string_view::npos) [[unlikely]]
            break;
        pnezd.point[0] = strtodd(range.data() + pos, NULL);
        pos = e + 1;

        // z
        size_t z = range.find(dlm, pos);
        if (z == std::string_view::npos) [[unlikely]]
            break;
        pnezd.point[2] = strtodd(range.data() + pos, NULL);
        pos = z + 1;

        size_t d = range.find('\r', pos);
        if (d != std::string_view::npos)
            pnezd._description = utf8_to_wstr(range.substr(pos, d - pos));
        else
            pnezd._description = utf8_to_wstr(range.substr(pos, len - pos));
        pos = d + 1;
        out.push_back(std::move(pnezd));
    }
}

bool parse_pnezd(const std::filesystem::path& inpath, char dlm, PNEZDArray& penzdList)
{
    try
    {
        FileHnd fh(CreateFile(inpath.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL));
        if (fh.hnd() == INVALID_HANDLE_VALUE)
            return false;

        LARGE_INTEGER fileSize;
        if (!GetFileSizeEx(fh.hnd(), &fileSize))
            return false;

        FileHnd fhm(CreateFileMapping(fh.hnd(), NULL, PAGE_READONLY, 0, 0, NULL));
        if (fhm.hnd() != INVALID_HANDLE_VALUE)
        {
            size_t nbytes = static_cast<size_t>(fileSize.QuadPart);
            FileMap mv(MapViewOfFile(fhm.hnd(), FILE_MAP_READ, 0, 0, nbytes), nbytes);
            auto segments = makefileSegments(mv.view());
            if (segments.size() == 1)
            {
                // Single-threaded parse
                parse_pnezd_range(segments[0], dlm, penzdList);
            }
            else
            {
                // Multi-threaded parse 
                std::vector<PNEZDArray> per_thread_results(segments.size());
                {
                    std::vector<std::jthread> jthreads;
                    jthreads.reserve(segments.size());
                    for (size_t i = 0; i < segments.size(); ++i)
                    {
                        const auto seg = segments[i];
                        jthreads.emplace_back([seg, &per_thread_results, i, dlm]()
                            {
                                parse_pnezd_range(seg, dlm, per_thread_results[i]);
                            });
                    }
                }

                // Merge results
                size_t total = 0;
                for (const auto& v : per_thread_results)
                    total += v.size();

                penzdList.reserve(total);
                for (auto& v : per_thread_results)
                {
                    for (auto& item : v)
                        penzdList.push_back(std::move(item));
                }
            }
        }
        return true;
    }
    catch (...)
    {
        acutPrintf(L"Exception parsing PNEZD file.\n");
    }
    return false;
}
