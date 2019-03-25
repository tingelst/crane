#ifndef __MLPICONTAINERHELPER_H__
#define __MLPICONTAINERHELPER_H__

// -----------------------------------------------------------------------
// MLPI - <mlpiContainerHelper.h>
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



//! @addtogroup UtilContainerHelper UtilContainerHelper
//! @ingroup Utilities
//! @{
//! @brief This module contains some useful functions and macros for container handling.
//!
//! @details
//! Please note, that this piece of source code is not directly part
//! of the MLPI. You do not need this file to program the
//! MLPI. Nevertheless, at least parts of this file have been considered
//! to be somewhat useful when using or learning to use MLPI functionality.
//! It is therefore included without any support, but to act as sample code
//! and source of inspiration.
//! @}



// -----------------------------------------------------------------------
// GLOBAL INCLUDES
// -----------------------------------------------------------------------
#include <string>
#include <set>
#include <map>
#include <vector>
#include <sstream>
#include <ostream>
#include <algorithm>
#include <numeric>
#include <memory>

#include "mlpiGlobal.h"
#include "wchar16.h"

#include "mlpiContainerLib.h"

// -----------------------------------------------------------------------
// GLOBAL TYPEDEFS
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// GLOBAL CONSTANTS
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// GLOBAL MACROS
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// GLOBAL EXPORTS
// -----------------------------------------------------------------------



#if defined(MLPI_ENABLE_EXPERIMENTAL)
// The following code is for experimental purpose only, and is not finished yet!!!
// Do not use it at the moment, but expect more soon! Comments are welcome...

class MlpiBaseContainer
{
public:

  // It is really annoying that there is no full support for std::wstring on VxWorks and Android.
  // And the biggest fail is, that the implementations are not compatible across VxWorks, Android
  // and Windows. So until we have our own MlpiString implementation, we have to stick with
  // the std::string ASCII version and do some more conversions :-(.
  typedef std::set<std::string> StringSet;
  typedef std::vector<std::string> StringVector;
  typedef std::map<std::string, MlpiContainerItemInformation> ItemMap;
  typedef std::pair<std::string, MlpiContainerItemInformation> StringItemPair;


  MlpiBaseContainer(MLPIHANDLE connection, MlpiContainerAccess accessFlag)
  {
    memset(&_hContainer, MLPI_INVALIDHANDLE, sizeof(_hContainer));
    _connection = connection;
    _bDirty = true;
    _containerSize = 0;
    _containerData = NULL;
    _accessFlag = accessFlag;
  }

  virtual ~MlpiBaseContainer()
  {
    Clear();
    Destroy();
  }

  MLPIRESULT AttachConnection(MLPIHANDLE connection)
  {
    if (connection == MLPI_INVALIDHANDLE)
      return MLPI_E_INVALIDARG;

    if (connection == _connection)
      return MLPI_S_OK;

    // Containers are bound to a connection. Thus, when attaching a new connection, we
    // have to destroy the old container.
    Destroy();

    _connection = connection;

    return MLPI_S_OK;
  }

  MLPIRESULT Clear()
  {
    m_tagSet.clear();
    m_itemMap.clear();
    _bDirty = true;

    return MLPI_S_OK;
  }

  MLPIRESULT Create()
  {
    return CheckContainer();
  }

  MLPIRESULT Destroy()
  {
    _bDirty = true;

    // delete old container
    delete [] _containerData;
    _containerData = NULL;
    if (_hContainer.containerID != MLPI_INVALIDHANDLE)
    {
      MLPIRESULT result = mlpiContainerDestroy(_connection, &_hContainer);
      if (MLPI_FAILED(result))
      {
        printf("\nERROR: could not destroy old container during destroy()!");
      }
      memset(&_hContainer, MLPI_INVALIDHANDLE, sizeof(_hContainer));
      _containerSize = 0;
    }

    return MLPI_S_OK;
  }

  MLPIRESULT Update()
  {
    MLPIRESULT result = CheckContainer();
    if (MLPI_FAILED(result))
      return result;

    // update the container
    result = mlpiContainerUpdate(_connection, _hContainer, _containerData, _containerSize);

    return result;
  }

  MLPIRESULT CheckContainer()
  {
    if (_bDirty)
    {
      // delete old container
      delete [] _containerData;
      _containerData = NULL;
      if (_hContainer.containerID != MLPI_INVALIDHANDLE)
      {
        MLPIRESULT result = mlpiContainerDestroy(_connection, &_hContainer);
        if (MLPI_FAILED(result))
        {
          printf("\nERROR: could not destroy old container during update()!");
        }
        memset(&_hContainer, MLPI_INVALIDHANDLE, sizeof(_hContainer));
        _containerSize = 0;
      }
      m_itemMap.clear();


      // create new container
      std::string description;
      description.reserve(m_tagSet.size()*64);

      printf("---Recreating Container---");
      for (StringSet::iterator iter=m_tagSet.begin(); iter!=m_tagSet.end(); iter++)
      {
        //printf("\nElement: %s", (*iter).c_str());
        description += *iter;
        description += MLPI_CONTAINER_TAG_SEPARATOR;
      }

      MLPIRESULT result = mlpiContainerCreate(_connection, A2W16(description.c_str()), _accessFlag, &_hContainer, &_containerSize);
      if (MLPI_FAILED(result))
      {
        printf("\nERROR: could not create new container during update()!");
        return result;
      }
      //printf("\n---Handle: 0x%08X Size: %ud---", _hContainer, _containerSize);

      _containerData = new UCHAR[_containerSize];

      if (_containerData == NULL) {
        Destroy();
        return MLPI_E_OUTOFMEMORY;
      }

      memset(_containerData, 0, _containerSize);


      // read container information and populate local members
      printf("\n---Building Map---");
      ULONG numReturned = 0;
      std::vector<MlpiContainerItemInformation> items(m_tagSet.size());

      mlpiContainerGetItemInformation(_connection, _hContainer, &items.at(0), static_cast<ULONG>(m_tagSet.size()), &numReturned);
      if (MLPI_FAILED(result))
      {
        printf("\nERROR: number of returned item informations does not match with sequence length!");
        Destroy();
        return result;
      }

      int i = 0;
      for (StringSet::iterator iter = m_tagSet.begin(); iter != m_tagSet.end(); iter++, i++)
      {
        MlpiContainerItemInformation elementInfo;
        elementInfo.dataSize = items.at(i).dataSize;
        elementInfo.offset   = items.at(i).offset;
        elementInfo.type     = items.at(i).type;
        m_itemMap.insert(StringItemPair(iter->c_str(), elementInfo));

        //printf("\nOffset: %03lu, Size: %03lu, Type %02d, %s", elementInfo.offset, elementInfo.dataSize, elementInfo.type,  descriptions.at(i).c_str());
      }

      _bDirty = false;
    }
    return MLPI_S_OK;
  }

  MLPIRESULT GetInformation(MlpiContainerInformation *containerInformation)
  {
    if (NULL == containerInformation)
      return MLPI_E_INVALIDARG;

    MLPIRESULT result = CheckContainer();
    if (MLPI_FAILED(result))
      return result;

    memset(&containerInformation, 0, sizeof(containerInformation));

    return mlpiContainerGetInformation(_connection, _hContainer, containerInformation);
  }

  MLPIRESULT GetTagList(WCHAR16 *tagList, const ULONG numElements)
  {
    if (tagList == NULL)
      return MLPI_E_INVALIDARG;

    MLPIRESULT result = CheckContainer();
    if (MLPI_FAILED(result))
      return result;

    return mlpiContainerGetTagList(_connection, _hContainer, tagList, numElements);
  }

  MLPIRESULT GetTagList(StringSet *tagList)
  {
    if (tagList == NULL)
      return MLPI_E_INVALIDARG;

    MLPIRESULT result = CheckContainer();
    if (MLPI_FAILED(result))
      return result;

    // Clear the list
    tagList->clear();

    // Get information of string length
    MlpiContainerInformation containerInformation;
    memset(&containerInformation, 0, sizeof(containerInformation));
    result = mlpiContainerGetInformation(_connection, _hContainer, &containerInformation);
    if (MLPI_FAILED(result))
      return result;

    // Read the string
    std::auto_ptr<WCHAR16> tagListString(new WCHAR16[containerInformation.numElementsTagList+1]);
    result = mlpiContainerGetTagList(_connection, _hContainer, tagListString.get(), containerInformation.numElementsTagList+1);
    if (MLPI_FAILED(result))
      return result;

    // Token list of strings to the given set
    Tokenize(*tagList, tagListString.get(), L";");

    return result;
  }

  MLPIRESULT Add(const WCHAR16 *tags)
  {
    if (tags == NULL)
      return MLPI_E_INVALIDARG;

    const WCHAR16 delimiter[] = {MLPI_CONTAINER_TAG_SEPARATOR, L'\0'};
    if (Tokenize(m_tagSet, tags, delimiter))
    {
      // new values inserted?
      _bDirty = true;
    }

    return MLPI_S_OK;
  }

  MLPIRESULT Remove(const WCHAR16 *tags)
  {
    if (tags == NULL)
      return MLPI_E_INVALIDARG;

    StringSet tagSet;
    const WCHAR16 delimiter[] = {MLPI_CONTAINER_TAG_SEPARATOR, L'\0'};
    if (Tokenize(tagSet, tags, delimiter))
    {
      for (StringSet::iterator iterLoop = tagSet.begin(); iterLoop != tagSet.end(); ++iterLoop)
      {
        StringSet::iterator iterFind = m_tagSet.find(*iterLoop);
        if (iterFind != m_tagSet.end())
        {
          m_tagSet.erase(iterFind);
          _bDirty = true;
        }
      }
    }

    return MLPI_S_OK;
  }

  MLPIRESULT Get(StringSet *tagList)
  {
    if (tagList == NULL)
      return MLPI_E_INVALIDARG;

    tagList->clear();
    tagList->insert(m_tagSet.begin(), m_tagSet.end());

    return MLPI_S_OK;
  }

  template<typename T>
  MLPIRESULT GetDataByItemInfo(const MlpiContainerItemInformation *itemInfo, T *data) const
  {
    if (data == NULL)
      return MLPI_E_INVALIDARG;

    const MlpiType type  = itemInfo->type;
    const ULONG offset   = itemInfo->offset;
    const ULONG dataSize = itemInfo->dataSize;

    if (offset + dataSize > _containerSize)
      return MLPI_E_FAIL;

    switch (type)
    {
    case MLPI_TYPE_CHAR_UTF8:
      {
        if (offset + 1 > _containerSize) return MLPI_E_FAIL;
        std::stringstream ss(reinterpret_cast<char*>(&_containerData[offset]));
        ss >> *data;
        if (ss.fail()) {
          return MLPI_E_TYPE_MISSMATCH;
        }
      }
      break;

    case MLPI_TYPE_CHAR_UTF16:
      {
        if (offset+sizeof(WCHAR16)>_containerSize) return MLPI_E_FAIL;
        std::stringstream ss(W2A16(reinterpret_cast<WCHAR16*>(&_containerData[offset])));
        ss >> *data;
        if (ss.fail()) {
          return MLPI_E_TYPE_MISSMATCH;
        }
      }
      break;

    case MLPI_TYPE_BOOL8:
      {
        if (offset+sizeof(BOOL8)>_containerSize) return MLPI_E_FAIL;
        *data = static_cast<T>( (*reinterpret_cast<UCHAR*>(&_containerData[offset])) ? TRUE : FALSE );
      }
      break;

    case MLPI_TYPE_CHAR:
      {
        if (offset+sizeof(CHAR)>_containerSize) return MLPI_E_FAIL;
        *data = static_cast<T>( *reinterpret_cast<CHAR*>(&_containerData[offset]) );
      }
      break;

    case MLPI_TYPE_UCHAR:
      {
        if (offset+sizeof(UCHAR)>_containerSize) return MLPI_E_FAIL;
        *data = static_cast<T>( *reinterpret_cast<UCHAR*>(&_containerData[offset]) );
      }
      break;

    case MLPI_TYPE_SHORT:
      {
        if (offset+sizeof(SHORT)>_containerSize) return MLPI_E_FAIL;
        *data = static_cast<T>( *reinterpret_cast<SHORT*>(&_containerData[offset]) );
      }
      break;

    case MLPI_TYPE_USHORT:
      {
        if (offset+sizeof(USHORT)>_containerSize) return MLPI_E_FAIL;
        *data = static_cast<T>( *reinterpret_cast<USHORT*>(&_containerData[offset]) );
      }
      break;

    case MLPI_TYPE_LONG:
      {
        if (offset+sizeof(LONG)>_containerSize) return MLPI_E_FAIL;
        *data = static_cast<T>( *reinterpret_cast<LONG*>(&_containerData[offset]) );
      }
      break;

    case MLPI_TYPE_ULONG:
      {
        if (offset + sizeof(ULONG) > _containerSize) return MLPI_E_FAIL;
        *data = static_cast<T>( *reinterpret_cast<ULONG*>(&_containerData[offset]) );
      }
      break;

    case MLPI_TYPE_LLONG:
      {
        if (offset + sizeof(LLONG)>_containerSize) return MLPI_E_FAIL;
        *data = static_cast<T>( *reinterpret_cast<LLONG*>(&_containerData[offset]) );
      }
      break;

    case MLPI_TYPE_ULLONG:
      {
        if (offset + sizeof(ULLONG)>_containerSize) return MLPI_E_FAIL;
        *data = static_cast<T>( *reinterpret_cast<ULLONG*>(&_containerData[offset]) );
      }
      break;

    case MLPI_TYPE_FLOAT:
      {
        if (offset + sizeof(FLOAT)>_containerSize) return MLPI_E_FAIL;
        *data = static_cast<T>( *reinterpret_cast<FLOAT*>(&_containerData[offset]) );
      }
      break;

    case MLPI_TYPE_DOUBLE:
      {
        if (offset + sizeof(DOUBLE)>_containerSize) return MLPI_E_FAIL;
        *data = static_cast<T>( *reinterpret_cast<DOUBLE*>(&_containerData[offset]) );
      }
      break;

    case MLPI_TYPE_CHAR_ARRAY:
    case MLPI_TYPE_UCHAR_ARRAY:
    case MLPI_TYPE_SHORT_ARRAY:
    case MLPI_TYPE_USHORT_ARRAY:
    case MLPI_TYPE_LONG_ARRAY:
    case MLPI_TYPE_ULONG_ARRAY:
    case MLPI_TYPE_LLONG_ARRAY:
    case MLPI_TYPE_ULLONG_ARRAY:
    case MLPI_TYPE_FLOAT_ARRAY:
    case MLPI_TYPE_DOUBLE_ARRAY:
    case MLPI_TYPE_BOOL8_ARRAY:
      // Cannot read array into scalar value
      return MLPI_E_SIZE_MISSMATCH;
      break;

    case MLPI_TYPE_INVALID:
    default:
      return MLPI_E_NOTSUPPORTED;
      break;
    }

    return MLPI_S_OK;
  }

  template<typename T>
  MLPIRESULT GetDataByItemInfo(const MlpiContainerItemInformation *itemInfo, std::vector<T> *data) const
  {
    if (data == NULL)
      return MLPI_E_INVALIDARG;

    const MlpiType type  = itemInfo->type;
    const ULONG offset   = itemInfo->offset;
    const ULONG dataSize = itemInfo->dataSize;

    if (offset + dataSize > _containerSize)
      return MLPI_E_FAIL;

    // always clear vector
    data->clear();

    switch (type)
    {
    case MLPI_TYPE_CHAR_UTF8:
      {
        if (offset + 1 > _containerSize) return MLPI_E_FAIL;
        std::stringstream ss(reinterpret_cast<char*>(&_containerData[offset]));
        T primitive;
        ss >> primitive;
        if (ss.fail()) {
          return MLPI_E_TYPE_MISSMATCH;
        }
        data->push_back(primitive);
      }
      break;

    case MLPI_TYPE_CHAR_UTF16:
      {
        if (offset+sizeof(WCHAR16)>_containerSize) return MLPI_E_FAIL;
        std::stringstream ss(W2A16(reinterpret_cast<WCHAR16*>(&_containerData[offset])));
        T primitive;
        ss >> primitive;
        if (ss.fail()) {
          return MLPI_E_TYPE_MISSMATCH;
        }
        data->push_back(primitive);
      }
      break;

    case MLPI_TYPE_BOOL8:
      {
        if (offset+sizeof(BOOL8)>_containerSize) return MLPI_E_FAIL;
        data->push_back( static_cast<T>( (*reinterpret_cast<UCHAR*>(&_containerData[offset])) ? TRUE : FALSE ) );
      }
      break;

    case MLPI_TYPE_CHAR:
      {
        if (offset+sizeof(CHAR)>_containerSize) return MLPI_E_FAIL;
        data->push_back( static_cast<T>( *reinterpret_cast<CHAR*>(&_containerData[offset]) ) );
      }
      break;

    case MLPI_TYPE_UCHAR:
      {
        if (offset+sizeof(UCHAR)>_containerSize) return MLPI_E_FAIL;
        data->push_back( static_cast<T>( *reinterpret_cast<UCHAR*>(&_containerData[offset]) ) );
      }
      break;

    case MLPI_TYPE_SHORT:
      {
        if (offset+sizeof(SHORT)>_containerSize) return MLPI_E_FAIL;
        data->push_back( static_cast<T>( *reinterpret_cast<SHORT*>(&_containerData[offset]) ) );
      }
      break;

    case MLPI_TYPE_USHORT:
      {
        if (offset+sizeof(USHORT)>_containerSize) return MLPI_E_FAIL;
        data->push_back( static_cast<T>( *reinterpret_cast<USHORT*>(&_containerData[offset]) ) );
      }
      break;

    case MLPI_TYPE_LONG:
      {
        if (offset+sizeof(LONG)>_containerSize) return MLPI_E_FAIL;
        data->push_back( static_cast<T>( *reinterpret_cast<LONG*>(&_containerData[offset]) ) );
      }
      break;

    case MLPI_TYPE_ULONG:
      {
        if (offset + sizeof(ULONG) > _containerSize) return MLPI_E_FAIL;
        data->push_back( static_cast<T>( *reinterpret_cast<ULONG*>(&_containerData[offset]) ) );
      }
      break;

    case MLPI_TYPE_LLONG:
      {
        if (offset + sizeof(LLONG)>_containerSize) return MLPI_E_FAIL;
        data->push_back( static_cast<T>( *reinterpret_cast<LLONG*>(&_containerData[offset]) ) );
      }
      break;

    case MLPI_TYPE_ULLONG:
      {
        if (offset + sizeof(ULLONG)>_containerSize) return MLPI_E_FAIL;
        data->push_back( static_cast<T>( *reinterpret_cast<ULLONG*>(&_containerData[offset]) ) );
      }
      break;

    case MLPI_TYPE_FLOAT:
      {
        if (offset + sizeof(FLOAT)>_containerSize) return MLPI_E_FAIL;
        data->push_back( static_cast<T>( *reinterpret_cast<FLOAT*>(&_containerData[offset]) ) );
      }
      break;

    case MLPI_TYPE_DOUBLE:
      {
        if (offset + sizeof(DOUBLE)>_containerSize) return MLPI_E_FAIL;
        data->push_back( static_cast<T>( *reinterpret_cast<DOUBLE*>(&_containerData[offset]) ) );
      }
      break;

    case MLPI_TYPE_CHAR_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(CHAR);
        const CHAR *first       = reinterpret_cast<CHAR*>(&_containerData[offset]);

        data->resize(numElements);
        std::transform(first, first + numElements, data->begin(), OperatorConvert<CHAR, T> );
      }
      break;

    case MLPI_TYPE_UCHAR_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(UCHAR);
        const UCHAR *first      = reinterpret_cast<UCHAR*>(&_containerData[offset]);

        data->resize(numElements);
        std::transform(first, first + numElements, data->begin(), OperatorConvert<UCHAR, T> );
      }
      break;

    case MLPI_TYPE_SHORT_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(SHORT);
        const SHORT *first      = reinterpret_cast<SHORT*>(&_containerData[offset]);

        data->resize(numElements);
        std::transform(first, first + numElements, data->begin(), OperatorConvert<SHORT, T> );
      }
      break;

    case MLPI_TYPE_USHORT_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(USHORT);
        const USHORT *first     = reinterpret_cast<USHORT*>(&_containerData[offset]);

        data->resize(numElements);
        std::transform(first, first + numElements, data->begin(), OperatorConvert<USHORT, T> );
      }
      break;

    case MLPI_TYPE_LONG_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(LONG);
        const LONG *first       = reinterpret_cast<LONG*>(&_containerData[offset]);

        data->resize(numElements);
        std::transform(first, first + numElements, data->begin(), OperatorConvert<LONG, T> );
      }
      break;

    case MLPI_TYPE_ULONG_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(ULONG);
        const ULONG *first      = reinterpret_cast<ULONG*>(&_containerData[offset]);

        data->resize(numElements);
        std::transform(first, first + numElements, data->begin(), OperatorConvert<ULONG, T> );
      }
      break;

    case MLPI_TYPE_LLONG_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(LLONG);
        const LLONG *first      = reinterpret_cast<LLONG*>(&_containerData[offset]);

        data->resize(numElements);
        std::transform(first, first + numElements, data->begin(), OperatorConvert<LLONG, T> );
      }
      break;

    case MLPI_TYPE_ULLONG_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(ULLONG);
        const ULLONG *first     = reinterpret_cast<ULLONG*>(&_containerData[offset]);

        data->resize(numElements);
        std::transform(first, first + numElements, data->begin(), OperatorConvert<ULLONG, T> );
      }
      break;

    case MLPI_TYPE_FLOAT_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(FLOAT);
        const FLOAT *first      = reinterpret_cast<FLOAT*>(&_containerData[offset]);

        data->resize(numElements);
        std::transform(first, first + numElements, data->begin(), OperatorConvert<FLOAT, T> );
      }
      break;

    case MLPI_TYPE_DOUBLE_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(DOUBLE);
        const DOUBLE *first     = reinterpret_cast<DOUBLE*>(&_containerData[offset]);

        data->resize(numElements);
        std::transform(first, first + numElements, data->begin(), OperatorConvert<DOUBLE, T> );
      }
      break;

    case MLPI_TYPE_BOOL8_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(BOOL8);
        const BOOL8 *first      = reinterpret_cast<BOOL8*>(&_containerData[offset]);

        data->resize(numElements);
        std::transform(first, first + numElements, data->begin(), OperatorConvert<BOOL8, T> );
      }
      break;

    case MLPI_TYPE_INVALID:
    default:
      return MLPI_E_NOTSUPPORTED;
      break;
    }

    return MLPI_S_OK;
  }

  MLPIRESULT GetDataByItemInfo(const MlpiContainerItemInformation *itemInfo, std::string *data) const
  {
    if (data == NULL)
      return MLPI_E_INVALIDARG;

    const MlpiType type  = itemInfo->type;
    const ULONG offset   = itemInfo->offset;
    const ULONG dataSize = itemInfo->dataSize;

    if (offset + dataSize > _containerSize)
      return MLPI_E_FAIL;

    // always clear vector
    data->clear();

    switch (type)
    {
    case MLPI_TYPE_CHAR_UTF8:
      {
        *data = std::string(reinterpret_cast<CHAR*>(&_containerData[offset]));
      }
      break;

    case MLPI_TYPE_CHAR_UTF16:
      {
        *data = std::string(W2A16(reinterpret_cast<WCHAR16*>(&_containerData[offset])));
      }
      break;

    case MLPI_TYPE_BOOL8:
      {
        if (offset+sizeof(BOOL8)>_containerSize) return MLPI_E_FAIL;
        *data = ToStdString<BOOL8>(reinterpret_cast<BOOL8*>(&_containerData[offset]));
      }
      break;

    case MLPI_TYPE_CHAR:
      {
        if (offset+sizeof(CHAR)>_containerSize) return MLPI_E_FAIL;
        *data = ToStdString(reinterpret_cast<CHAR*>(&_containerData[offset]));
      }
      break;

    case MLPI_TYPE_UCHAR:
      {
        if (offset+sizeof(UCHAR)>_containerSize) return MLPI_E_FAIL;
        *data = ToStdString(reinterpret_cast<UCHAR*>(&_containerData[offset]));
      }
      break;

    case MLPI_TYPE_SHORT:
      {
        if (offset+sizeof(SHORT)>_containerSize) return MLPI_E_FAIL;
        *data = ToStdString(reinterpret_cast<SHORT*>(&_containerData[offset]));
      }
      break;

    case MLPI_TYPE_USHORT:
      {
        if (offset+sizeof(USHORT)>_containerSize) return MLPI_E_FAIL;
        *data = ToStdString(reinterpret_cast<USHORT*>(&_containerData[offset]));
      }
      break;

    case MLPI_TYPE_LONG:
      {
        if (offset+sizeof(LONG)>_containerSize) return MLPI_E_FAIL;
        *data = ToStdString(reinterpret_cast<LONG*>(&_containerData[offset]));
      }
      break;

    case MLPI_TYPE_ULONG:
      {
        if (offset + sizeof(ULONG) > _containerSize) return MLPI_E_FAIL;
        *data = ToStdString(reinterpret_cast<ULONG*>(&_containerData[offset]));
      }
      break;

    case MLPI_TYPE_LLONG:
      {
        if (offset + sizeof(LLONG)>_containerSize) return MLPI_E_FAIL;
        *data = ToStdString<LLONG>(reinterpret_cast<LLONG*>(&_containerData[offset]));
      }
      break;

    case MLPI_TYPE_ULLONG:
      {
        if (offset + sizeof(ULLONG)>_containerSize) return MLPI_E_FAIL;
        *data = ToStdString(reinterpret_cast<ULLONG*>(&_containerData[offset]));
      }
      break;

    case MLPI_TYPE_FLOAT:
      {
        if (offset + sizeof(FLOAT)>_containerSize) return MLPI_E_FAIL;
        *data = ToStdString(reinterpret_cast<FLOAT*>(&_containerData[offset]));
      }
      break;

    case MLPI_TYPE_DOUBLE:
      {
        if (offset + sizeof(DOUBLE)>_containerSize) return MLPI_E_FAIL;
        *data = ToStdString<DOUBLE>(reinterpret_cast<DOUBLE*>(&_containerData[offset]));
      }
      break;

    case MLPI_TYPE_CHAR_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(CHAR);
        const CHAR *first       = reinterpret_cast<CHAR*>(&_containerData[offset]);

        *data = CutTrailingSpace(std::accumulate(first, first + numElements, std::string(), OperatorConvertArrayToString<CHAR>));
      }
      break;

    case MLPI_TYPE_UCHAR_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(UCHAR);
        const UCHAR *first      = reinterpret_cast<UCHAR*>(&_containerData[offset]);

        *data = CutTrailingSpace(std::accumulate(first, first + numElements, std::string(), OperatorConvertArrayToString<UCHAR>));
      }
      break;

    case MLPI_TYPE_SHORT_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(SHORT);
        const SHORT *first      = reinterpret_cast<SHORT*>(&_containerData[offset]);

        *data = CutTrailingSpace(std::accumulate(first, first + numElements, std::string(), OperatorConvertArrayToString<SHORT>));
      }
      break;

    case MLPI_TYPE_USHORT_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(USHORT);
        const USHORT *first     = reinterpret_cast<USHORT*>(&_containerData[offset]);

        *data = CutTrailingSpace(std::accumulate(first, first + numElements, std::string(), OperatorConvertArrayToString<USHORT>));
      }
      break;

    case MLPI_TYPE_LONG_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(LONG);
        const LONG *first       = reinterpret_cast<LONG*>(&_containerData[offset]);

        *data = CutTrailingSpace(std::accumulate(first, first + numElements, std::string(), OperatorConvertArrayToString<LONG>));
      }
      break;

    case MLPI_TYPE_ULONG_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(ULONG);
        const ULONG *first      = reinterpret_cast<ULONG*>(&_containerData[offset]);

        *data = CutTrailingSpace(std::accumulate(first, first + numElements, std::string(), OperatorConvertArrayToString<ULONG>));
      }
      break;

    case MLPI_TYPE_LLONG_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(LLONG);
        const LLONG *first      = reinterpret_cast<LLONG*>(&_containerData[offset]);

        *data = CutTrailingSpace(std::accumulate(first, first + numElements, std::string(), OperatorConvertArrayToString<LLONG>));
      }
      break;

    case MLPI_TYPE_ULLONG_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(ULLONG);
        const ULLONG *first     = reinterpret_cast<ULLONG*>(&_containerData[offset]);

        *data = CutTrailingSpace(std::accumulate(first, first + numElements, std::string(), OperatorConvertArrayToString<ULLONG>));
      }
      break;

    case MLPI_TYPE_FLOAT_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(FLOAT);
        const FLOAT *first      = reinterpret_cast<FLOAT*>(&_containerData[offset]);

        *data = CutTrailingSpace(std::accumulate(first, first + numElements, std::string(), OperatorConvertArrayToString<FLOAT>));
      }
      break;

    case MLPI_TYPE_DOUBLE_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(DOUBLE);
        const DOUBLE *first     = reinterpret_cast<DOUBLE*>(&_containerData[offset]);

        *data = CutTrailingSpace(std::accumulate(first, first + numElements, std::string(), OperatorConvertArrayToString<DOUBLE>));
      }
      break;

    case MLPI_TYPE_BOOL8_ARRAY:
      {
        const ULONG numElements = dataSize / sizeof(BOOL8);
        const BOOL8 *first      = reinterpret_cast<BOOL8*>(&_containerData[offset]);

        *data = CutTrailingSpace(std::accumulate(first, first + numElements, std::string(), OperatorConvertArrayToString<BOOL8>));
      }
      break;

    case MLPI_TYPE_INVALID:
    default:
      return MLPI_E_NOTSUPPORTED;
      break;
    }

    return MLPI_S_OK;
  }

  template <typename T>
  MLPIRESULT GetDataByTag(const WCHAR16 *tag, T *data) const
  {
    const ItemMap::const_iterator iter = m_itemMap.find(W2A16(tag));
    if (iter == m_itemMap.end())
      return MLPI_E_INVALIDARG;

    return GetDataByItemInfo(&iter->second, data);
  }

  template <typename T>
  MLPIRESULT GetDataByTag(const WCHAR16 *tag, std::vector<T> *data) const
  {
    const ItemMap::const_iterator iter = m_itemMap.find(W2A16(tag));
    if (iter == m_itemMap.end())
      return MLPI_E_INVALIDARG;

    return GetDataByItemInfo(&iter->second, data);
  }

  MLPIRESULT GetDataPointer(UCHAR **data, ULONG *size) const
  {
    if (data == NULL)
      return MLPI_E_INVALIDARG;
    *data = _containerData;

    if (size != NULL)
      *size = _containerSize;
  }

  MLPIRESULT GetItemInfoByTag(const WCHAR16 *tag, MlpiContainerItemInformation *itemInfo) const
  {
    if (itemInfo == NULL)
      return MLPI_E_INVALIDARG;

    const ItemMap::const_iterator iter = m_itemMap.find(W2A16(tag));
    if (iter == m_itemMap.end())
      return MLPI_E_INVALIDARG;

    // return a copy of the item info
    *itemInfo = iter->second;

    return MLPI_S_OK;
  }


private:
  // disallow copy and assignment
  MlpiBaseContainer(const MlpiBaseContainer& container);
  MlpiBaseContainer& operator=(const MlpiBaseContainer& container);

  static bool Tokenize(StringSet &list, const WCHAR16 *data, const WCHAR16 *delimiter)
  {
    bool insertedNew = false;
    std::auto_ptr<WCHAR16> dataCopy(wcsdup16(data));

    // token list of strings and try to add each element to the given string set
    WCHAR16 *context = NULL;
    WCHAR16 *token = wcstok_r16(dataCopy.get(), delimiter, &context);
    while(token != NULL)
    {
      if (list.insert(W2A16(token)).second == true)
        insertedNew = true;

      token = wcstok_r16(NULL, delimiter, &context);
    }

    // return status if any new items have been added to the string set
    return insertedNew;
  }

  template <typename Tin, typename Tout>
  static Tout OperatorConvert(Tin value)
  {
    return static_cast<Tout>(value);
  }

  template <typename Tin>
  static std::string OperatorConvertArrayToString(std::string value1, Tin value2)
  {
    return value1 + ToStdString<Tin>(&value2) + " ";
  }

  template <typename T>
  static std::string ToStdString(T *data)
  {
    std::ostringstream str;
    str << *data;
    return std::string(str.str());
  }

  static std::string ToStdString(UCHAR *data)
  {
    std::ostringstream str;
    str << static_cast<USHORT>(*data);
    return std::string(str.str());
  }

  static std::string ToStdString(CHAR *data)
  {
    std::ostringstream str;
    str << static_cast<SHORT>(*data);
    return std::string(str.str());
  }

  static std::string ToStdString(BOOL8 *data)
  {
    return std::string( (*reinterpret_cast<UCHAR*>(data)) ? "TRUE" : "FALSE" );
  }

  static std::string CutTrailingSpace(std::string value)
  {
    if (value.size() > 0)
    {
      std::string::iterator iterEnd = value.end() - 1;
      if (*iterEnd == ' ')
      {
        value.erase(iterEnd);
      }
    }

    return value;
  }

  MLPIHANDLE _connection;
  MlpiContainerAccess _accessFlag;
  MlpiContainerHandle _hContainer;
  ULONG _containerSize;
  UCHAR *_containerData;
  bool _bDirty;

  StringSet m_tagSet;
  ItemMap m_itemMap;
};


class MlpiReadContainer : public MlpiBaseContainer
{
public:
  MlpiReadContainer(MLPIHANDLE connection)
    : MlpiBaseContainer(connection, MLPI_CONTAINER_ACCESS_READ)
  {

  }
  ~MlpiReadContainer()
  {

  }

private:
};

#endif


/*
==============================================================================
History
------------------------------------------------------------------------------
01-Jan-2012
  - first version
==============================================================================
*/

#endif /* __MLPICONTAINERHELPER_H__ */
