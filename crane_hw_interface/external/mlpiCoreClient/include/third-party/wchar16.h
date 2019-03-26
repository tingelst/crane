#ifndef __WCHAR16_H__
#define __WCHAR16_H__

// -----------------------------------------------------------------------
// MLPI - <wchar16.h>
// -----------------------------------------------------------------------
// Copyright (c) 2013 Bosch Rexroth. All rights reserved.
// Redistribution and use in source and binary forms of this MLPI software
// (SW) provided to you, with or without modification, are permitted
// without prior approval provided that the following conditions are met:
//
// 1. Redistributions of source code of SW must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form of SW must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. User recognizes and acknowledges that it acquires no right,
// title or interest in or to any of the names or trademarks used in
// connection with the SW ("names") by virtue of this License and waives
// any right to or interest in the names. User recognizes and acknowledges
// that names of companies or names or products of companies displayed
// in the documentation of SW to indicate the interoperability of products
// with the SW are the names of their respective owners. The use of such
// names in the documentation of SW does not imply any sponsorship,
// approval, or endorsement by such companies of this product.
//
// 4. Modified code versions, i.e. any addition to or deletion from
// the substance or structure of the original code of the SW running
// the MLPI must be plainly marked as such and must not be misrepresented
// as being original SW.
//
// 5. The SW may only be used in connection with a Bosch Rexroth product.
//
// THIS INFORMATION IS PROVIDED BY BOSCH REXROTH CORPORATION "AS IS"
// AND WITHOUT WARRANTY OF ANY KIND, EXPRESSED OR IMPLIED, INCLUDING
// (BUT NOTLIMITED TO) ANY IMPLIED WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR ANY PARTICULAR PURPOSE, OR NON-INFRINGEMENT. WHILE THE
// INFORMATION PROVIDED IS BELIEVED TO BE ACCURATE, IT MAY INCLUDE
// ERRORS OR INACCURACIES.
// SUBJECT TO COMPULSORY STATUTORY PROVISIONS OF THE GERMAN LAW AS
// THE APPLICABLE LAW FOR THIS LICENSE BOSCH REXROTH CORPORATION WILL
// NOT BE LIABLE FOR ANY DAMAGES OF ANY KIND ARISING FROM THE USE OF
// THE  SOFTWARE  DISTRIBUTED HEREUNDER, INCLUDING BUT NOT LIMITED TO
// DIRECT, INDIRECT, INCIDENTAL, PUNITIVE, AND CONSEQUENTIAL DAMAGES.
// -----------------------------------------------------------------------
//
//! @file
//!
//! @author     DC-IA/EAM1 (SK, JR)
//!
//! @copyright  Bosch Rexroth Corporation http://www.boschrexroth.com/oce
//!
//! @version    1.22.0
//!
//! @date       2013
//
// -----------------------------------------------------------------------




//! @addtogroup UtilWchar16 UtilWchar16
//! @ingroup Utilities
//! @{
//! @brief This module contains some useful functions and macros for handling of WCHAR16 strings.
//! Use these routines on platforms which have no built-in support for <wchar.h> library or when
//! sizeof(wchar_t) != 2.
//!
//! @details
//! Please note that this piece of source code is not directly part
//! of the MLPI. You do not need this file to program against the
//! MLPI. Nevertheless, at least parts of this file have been considered
//! to be somewhat useful when using or learning to use MLPI functionality.
//! It is therefore included without any support, but to act as sample code
//! and source of inspiration.
//! @}



// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include <string.h>

#include "mlpiGlobal.h"

// -----------------------------------------------------------------------
// GLOBAL MACROS
// -----------------------------------------------------------------------
#if defined(TARGET_COMPILER_GCC)
  // compiling with GNU gcc
  #define W2A16(strIn) (_wcstombs16((CHAR*)__builtin_alloca(sizeof(CHAR)*(wcslen16(strIn)+1)), strIn, wcslen16(strIn)+1))
  #define A2W16(strIn) (_mbstowcs16((WCHAR16*)__builtin_alloca(sizeof(WCHAR16)*(strlen(strIn)+1)), strIn, strlen(strIn)+1))
#elif defined(TARGET_COMPILER_MSVC)
  // compiling with Microsoft Visual Studio Compiler
  #include "malloc.h"
  #define W2A16(strIn) (_wcstombs16((CHAR*)alloca(sizeof(CHAR)*(wcslen16(strIn)+1)), strIn, wcslen16(strIn)+1))
  #define A2W16(strIn) (_mbstowcs16((WCHAR16*)alloca(sizeof(WCHAR16)*(strlen(strIn)+1)), strIn, strlen(strIn)+1))
#else
  // unknown compiler
  #pragma warning "unknown compiler"
#endif


// -----------------------------------------------------------------------
// GLOBAL EXPORTS
// -----------------------------------------------------------------------

//! @ingroup UtilWchar16
//! Function determines the length of the WCHAR16 string (without the NULL terminator).
//! @param[in]  src   WCHAR16 string.
//! @return           The length of the WCHAR16 string.
inline size_t wcslen16(const WCHAR16 *src)
{
  const WCHAR16 *s;

  if(src == NULL)
    return 0;

  for (s = src; *s; ++s)
    continue;

  return (s - src);
}

//! @ingroup UtilWchar16
//! Function searches a WCHAR16 character in a WCHAR16 string.
//! @param[in]  src        WCHAR16 string.
//! @param[in]  character  WCHAR16 character to be located.
//! @return                Pointer to the first occurrence of character in src. If character is not found, the function returns a NULL pointer.
inline WCHAR16 *wcschr16(const WCHAR16 *src, WCHAR16 character)
{
  const WCHAR16 *p;

  p = NULL;
  do
  {
    if (*src == character)
    {
      p = src;
    }
  } while (*src++);

  return const_cast<WCHAR16 *>(p);
}

//! @ingroup UtilWchar16
//! Function compares two WCHAR16 strings up to a certain length with each other.
//! @param[in]  src1  WCHAR16 string to be compared.
//! @param[in]  src2  WCHAR16 string to be compared.
//! @param[in]  len   Maximum number of characters to compared
//! @return           Returns an integral value indicating the relationship between the WCHAR16 strings:
//!                   <BR>A zero value indicates that the characters compared in both strings form the same string.
//!                   <BR>A value greater than zero indicates that the first character that does not match has a greater value in src1 than in src2;
//!                   And a value less than zero indicates the opposite.
inline int wcsncmp16(const WCHAR16 *src1, const WCHAR16 *src2, size_t len)
{
  while(len--)
  {
    if(*src1++ != *src2++)
    {
      return *(WCHAR16*)(src1 - 1) - *(WCHAR16*)(src2 - 1);
    }
  }

  return 0;
}

//! @ingroup UtilWchar16
//! Function compares two WCHAR16 strings with each other.
//! @param[in]  src1  WCHAR16 sting to be compared.
//! @param[in]  src2  WCHAR16 sting to be compared.
//! @return           Returns an integral value indicating the relationship between the WCHAR16 strings:
//!                   <BR>A zero value indicates that both are considered equal.
//!                   <BR>A value greater than zero indicates that the first WCHAR16 character that does not match has a greater value in src1 than in src2;
//!                   And a value less than zero indicates the opposite.
inline int wcscmp16(const WCHAR16 *src1, const WCHAR16 *src2)
{
  while (*src1 == *src2++)
  {
    if (!*src1++)
    {
      return 0;
    }
  }

  return src1[0] - src2[-1];
}

inline int wcscmp16_(const WCHAR16 *src1, const WCHAR16 *src2)
{
  while ((~0x0020 & *src1) == (~0x0020 & *src2++))    // ignore case sensitivity...
  {
    if (!*src1++)
    {
      return 0;
    }
  }

  return src1[0] - src2[-1];
}

//! @ingroup UtilWchar16
//! Function copies a WCHAR16 string.
//! @param[out]  dst    Pointer to the destination array, where the content is to be copied.
//! @param[in]   src    WCHAR16 string to be copied.
//! @return             Pointer to destination string is returned.
inline WCHAR16* wcscpy16(WCHAR16 *dst, const WCHAR16 *src)
{
  WCHAR16 *run = dst;
  while ((*run++ = *src++) != L'\0');

  return dst;
}

//! @ingroup UtilWchar16
//! Function copies a specified number of WCHAR16 characters of a WCHAR16 string.
//! @param[out]  dst    Pointer to the destination array, where the content is to be copied.
//! @param[in]   src    WCHAR16 string to be copied.
//! @param[in]   len    Maximum number of characters to be copied from src.
//! @return             Pointer to destination string is returned.
inline WCHAR16* wcsncpy16(WCHAR16 *dst, const WCHAR16 *src, size_t len)
{
  if (len!=0)
  {
    int i = 0;
    while(len != 0) {
      len--;
      if ((dst[i++] = *src++) == 0)
        break;
    };
    while(len-- != 0)
      dst[i++] = 0;
  }
  return dst;
}

//! @ingroup UtilWchar16
//! Function appends a WCHAR16 string to another.
//! @param[out]  dst    Pointer to the destination array, which should contain a WCHAR16 string, and be large enough to contain the concatenated
//!                     resulting string, including the additional NULL terminator.
//! @param[in]   src    WCHAR16 string to be appended. This should not overlap dst.
//! @return             Pointer to destination string is returned.
inline WCHAR16* wcscat16(WCHAR16 *dst, const WCHAR16 *src)
{
  int i,j;
  for (i = 0; dst[i] != '\0'; i++)
    ;
  for (j = 0; src[j] != '\0'; j++)
    dst[i+j] = src[j];
  dst[i+j] = '\0';

  return dst;
}

//! @ingroup UtilWchar16
//! Function appends a WCHAR16 string with a certain length to another.
//! @param[out]  dst    Pointer to the destination array, which should contain a WCHAR16 string, and be large enough to contain the concatenated
//!                     resulting string, including the additional NULL terminator.
//! @param[in]   src    WCHAR16 string to be appended.
//! @param[in]   len    Maximum number of characters to be appendend.
//! @return             Pointer to destination string is returned.
inline WCHAR16* wcsncat16(WCHAR16 *dst, const WCHAR16 *src, size_t len)
{
  int i = 0;

  while (dst[i++] != 0)
    ;
  if (len != 0) {
    i--;
    do {
      if ((dst[i++] = *src++) == 0)
        break;
    } while (--len != 0);
  }
  return dst;
}

//! @ingroup UtilWchar16
//! Function appends a WCHAR16 string with a certain length to another.
//! @param[in]  src   WCHAR16 string.
//! @return           Pointer to the storage location for the copied string or NULL if storage cannot be allocated.
inline WCHAR16* wcsdup16(const WCHAR16 *src)
{
  WCHAR16 *dst = new WCHAR16[wcslen16(src) + sizeof(WCHAR16)];
  if(dst != NULL)
    wcscpy16(dst, src);

  return dst;
}

//! @ingroup UtilWchar16
//! Function to determine the positon of one string in another.
//! @param[in]  s     WCHAR16 string to be scanned.
//! @param[in]  t     WCHAR16 string to be searched in s.
//! @param[in]  pos   Position in s where the search should start.
//! @return           The position of the first character of the first match of t in s after the startpositon. If a string is empty or the startpositon is
//!                   smaller than zero, -2 is returned. If the function call failed, -1 is returned.
inline int wcsfind16(const WCHAR16 *s, const WCHAR16 *t, const int pos=0)
{
  int i = 0, j;
  if ( (s==NULL) || (t==NULL) || (pos<0) ) return -2;
  while (s[pos+i] != 0)
  {
    j=0;
    while ( (s[pos+i+j] != 0) && (s[pos+i+j] == t[j]) )
    {
      if (t[j+1] == 0)
        return (i+j);
      j++;
    }
    i++;
    if(i < 0) return -3;
  }

  return -1;
}

//! @ingroup UtilWchar16
//! Function to determine the length of the matching WCHAR16 string.
//! @param[in]  src1  WCHAR16 string to be scanned.
//! @param[in]  src2  WCHAR16 string containing the characters to match.
//! @return           The length of the initial portion of src1 containing only WCHAR16 charactes that appear in src2. Therefore, if all of the WCHAR16 characters
//!                   in src1 are in src2, the function returns the length of the entire src1 WCHAR16 string, and if the first WCHAR16 in src1 is not in src2,
//!                   the function returns zero.
inline int wcsspn16(const WCHAR16 *src1, const WCHAR16 *src2)
{
  const WCHAR16 *s = src1;
  const WCHAR16 *p = src2;

  while (*p)
  {
    if (*p++ == *s)
    {
      ++s;
      p = src2;
    }
  }

  return (int)(s - src1);
}

//! @ingroup UtilWchar16
//! Function scans WCHAR16 strings for characters in specified character sets.
//! @param[in]  src1  WCHAR16 string to be scanned.
//! @param[in]  src2  WCHAR16 string containing the characters to match.
//! @return           Pointer to the first occurrence of any character from src2 in src1, or a NULL pointer if the two string arguments have no characters in common.
inline WCHAR16* wcspbrk16(const WCHAR16 *src1, const WCHAR16 *src2)
{
  const WCHAR16 *s;
  const WCHAR16 *p;

  for (s=src1; *s; s++)
  {
    for (p=src2; *p; p++)
    {
      if (*p == *s) return (WCHAR16 *) s;
    }
  }

  return NULL;
}

//! @ingroup UtilWchar16
//! Function to disassemble a WCHAR16 string for certain WCHAR16 characters.
//! A sequence of calls to this function split s1 into tokens, which are sequences of contiguous WCHAR16 characters separated by any of the WCHAR16 characters that
//! are part of the delimiter.
//! @param[in]   s1      WCHAR16 string containing token or tokens.
//! @param[in]   s2      Set of delimiter characters.
//! @param[out]  context Used to store position information.
//! @return              Pointer to the next Token found in s1. NULL is returned when no more tokens are found. Each call modifies s1 by substituting a NULL character
//!                      for the first delimiter that occurs after the returned token.
inline WCHAR16* wcstok_r16(WCHAR16 *s1, const WCHAR16 *s2, WCHAR16 **context)
{
  WCHAR16 *s;
  WCHAR16 *p;

  if (((s = s1) != NULL) || ((s = *context) != NULL))
  {
    if (*(s += wcsspn16(s, s2)))
    {
      if ((p = wcspbrk16(s, s2)) != NULL)
      {
        *p++ = 0;
      }
    } else
    {
      p = s = NULL;
    }
    *context = p;
  }

  return s;
}

//! @ingroup UtilWchar16
//! Function searches for a specific WCHAR16 string sequence in a WCHAR16 string.
//! @param[in]  src1  WCHAR16 string to be scanned.
//! @param[in]  src2  WCHAR16 string containing the characters to match.
//! @return           Pointer to the first occurrence in src1 of the entire sequence of characters specified in src2, or NULL if the sequence is not present in src1.
inline WCHAR16 *wcsstr16(WCHAR16 *src1, WCHAR16 *src2)
{
  size_t n = wcslen16(src2);
  while(*src1)
    if(!memcmp(src1++, src2, sizeof(WCHAR16) * n))
      return src1 - 1;

  return 0;
}

//! @ingroup UtilWchar16
//! Function searches for a specific WCHAR16 string sequence in a WCHAR16 string.
//! @param[in]  src1  WCHAR16 string to be scanned.
//! @param[in]  src2  WCHAR16 string containing the characters to match.
//! @return           Pointer to the first occurrence in src1 of the entire sequence of characters specified in src2, or NULL if the sequence is not present in src1.
inline const WCHAR16 *wcsstr16(const WCHAR16 *src1, const WCHAR16 *src2)
{
  size_t n = wcslen16(src2);
  while(*src1)
    if(!memcmp(src1++, src2, sizeof(WCHAR16) * n))
      return src1 - 1;

  return 0;
}

//! @ingroup UtilWchar16
//! Function converts a WCHAR16 string to integer.
//! @param[in]  src  WCHAR16 string to be converted.
//! @return          Integer value produced by interpreting the input characters as a number. The return value is 0, if the input cannot be converted to integer.
inline int wtoi16(const WCHAR16 *src)
{
  int result = 0;
  while (*src >= '0' && *src <= '9')
    result = result * 10 + *src++ - '0';

  return result;
}

//! @ingroup UtilWchar16
//! Function translates WCHAR16 characters from the sequence pointed by src to the multibyte equivalent sequence (which is stored at the array pointed by dst),
//! up until either len bytes have been translated or until a WCHAR16 character translates into a NULL character.
//! @param[out]  dst  Pointer to an array of char elements long enough to contain the resulting sequence.
//! @param[in]   src  WCHAR16 string to be translated.
//! @param[in]   len  Maximum number of bytes to be written to dst.
//! @return           The number of bytes written to dst, not including the eventual ending NULL-character. If a WCHAR16 character that does not correspond to a
//!                   valid multibyte character is encountered, a -1 value is returned.
inline size_t wcstombs16(char *dst, const WCHAR16 *src, size_t len)
{
  size_t count = 0;

  if (len != 0)
  {
    do
    {
      if ((*dst++ = (char)*src++) == 0)
        break;
      count++;
    } while (--len != 0);
  }

  return count;
}

//! @ingroup UtilWchar16
//! Function converts a sequence of multibyte characters to a corresponding sequence of WCHAR16 characters.
//! @param[out]  dst  Pointer to the address of a sequence of WCHAR16 characters.
//! @param[in]   src  Pointer to the address of a sequence of mulitybyte characters.
//! @param[in]   len  The number of multibyte characters to convert.
//! @return           The number of converted multibyte characters. If the dst argument is NULL, the function returns the required size of the destination string.
//!                   If an invalid multibyte character is encountered, it returns -1.
inline size_t mbstowcs16(WCHAR16 *dst, const char *src, size_t len)
{
  size_t count = 0;

  if (len != 0)
  {
    do
    {
      if ((*dst++ = (WCHAR16)*src++) == 0)
        break;
      count++;
    } while (--len != 0);
  }

  return count;
}

//! @ingroup UtilWchar16
//! Function translates WCHAR16 characters from the sequence pointed by src to the multibyte equivalent sequence (which is stored at the array pointed by dst),
//! up until either len bytes have been translated or until a WCHAR16 character translates into a NULL character.
//! @param[out]  dst  Pointer to an array of char elements long enough to contain the resulting sequence.
//! @param[in]   src  WCHAR16 string to be translated.
//! @param[in]   len  Maximum number of bytes to be written to dst.
//! @return           Pointer to dst.
inline char* _wcstombs16(char *dst, const WCHAR16 *src, size_t len)
{
  // same as wcstombs16, but returns pointer to dst instead of length
  wcstombs16(dst, src, len);
  return dst;
}

//! @ingroup UtilWchar16
//! Function converts a sequence of multibyte characters to a corresponding sequence of WCHAR16 characters.
//! @param[out]  dst  Pointer to the address of a sequence of WCHAR16 characters.
//! @param[in]   src  Pointer to the address of a sequence of mulitybyte characters.
//! @param[in]   len  The number of multibyte characters to convert.
//! @return           Pointer to dst.
inline WCHAR16* _mbstowcs16(WCHAR16 *dst, const char *src, size_t len)
{
  // same as mbstowcs16, but returns pointer to dst instead of length
  mbstowcs16(dst, src, len);
  return dst;
}

//! @ingroup UtilWchar16
//! Function to convert a data type to a binary string. (UCHAR) 0xC => "1010"
//! @param[in]   src   This data (of type T) will be converted to a binary string.
//! @param[out]  dst   Destination to store string (must be greater than the number of bits src has).
//! @param[in]   len   Size of available storage.
//! @return            Pointer to the destination string.
template<typename T>
WCHAR16* itow16_binary(T src, WCHAR16* dst, const size_t len)
{
  size_t i = 0;
  const size_t numBits = sizeof(T) * 8;

  if(dst && len && (len > numBits + 1))
  {
    const T mask = 0x1;
    for(i = 0; i < numBits; i++)
    {
      T value = src & mask;
      dst[numBits - i - 1] = (value) ? '1' : '0';
      src = src >> 1;
    }
  }

  dst[i] = '\0';
  return dst;
}


/*
==============================================================================
History
------------------------------------------------------------------------------
01-Jan-2012
  - first version
14-Aug-2012
  - added wcscat16, wcsspn16, wcspbrk16, wcstok_r16
12-Dec-2012
  - fixed wcstombs16 which converted to UTF-8 and not to extended ASCII
    which is more convenient.
17-Jan-2013 SK
  - fixed implementation of wcsncmp16
  - added simple implementation of wcsstr16
25-Mar-2014 MER supervised by SK
  - added comments to each function
==============================================================================
*/

#endif /* __WCHAR16_H__ */
