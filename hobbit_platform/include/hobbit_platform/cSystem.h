//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 27.10.2010
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef C_SYSTEM_HH
#define C_SYSTEM_HH
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <new>
#include <string>
#include <inttypes.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "cException.h"
#include "cXml.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Predefined datatypes that PCData can be converted to.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define ELEMENT_UINT8                   0
#define ELEMENT_INT8                    1
#define ELEMENT_UINT16                  2
#define ELEMENT_INT16                   3
#define ELEMENT_UINT32                  4
#define ELEMENT_INT32                   5
#define ELEMENT_FLOAT                   6
#define ELEMENT_DOUBLE                  7
#define ELEMENT_STRING                  8
#define ELEMENT_BOOLEAN                 9
#define ELEMENT_NODE                    10
#define ELEMENT_OBJECT                  11
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// The class 'cSystemException'.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The class 'cSystemException' is derived from the base class
// 'cException'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cSystemException : public cException
{
  protected:
//-------------------------------------------------------------------
// Private attributes.
//-------------------------------------------------------------------
    string SystemErrorName;
  public:
//-------------------------------------------------------------------
// Public methods.
//-------------------------------------------------------------------
    cSystemException(uint32_t ID, const char *Method, const char *Error, const char *SystemError);
    const char *GetSystemError(void);
    void Print(void);
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// The class 'cSystem'.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// For every node with the name 'Object', the attributes 'interface',
// 'implementation' and 'id' an object is created (i.e. the instance
// of a class) and added to an object pool.
// Each entry in that object pool provides the following information:
// (1) a pointer to the object (instance of the class).
// (2) the type of the base class (interface)
// (3) the type of the derived class (implementation)
// (4) the ID of the object
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
struct ObjectPoolEntry
{
  void *pObject;
  const char *pInterface;
  const char *pImplementation;
  uint32_t ID;
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Methods that need to extract simple elements such as integer and
// float numbers from the XML file, always have to go through these
// steps: [1] get the (simple) start tag's name, [2] get the PCData
// and convert it to said numbers.
// In order to automatise these steps, the structure 'SimpleElement'
// is a descriptor that provides all information required: [A] the name
// of the attribute to be initialised with data from the XML file,
// [B] a pointer to the attribute, [C] the datatype of the attribute,
// [D] a flag telling if the attribute has already been initialised
// ('true') or not ('false') and [E] the maximum data size, which is
// only relevant if the data is a string.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
struct SimpleElement
{
  const char *TagName;
  void *Attribute;
  uint16_t DataType;
  bool bInitialised;
  uint32_t Size;
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cSystem
{
  protected:
//-------------------------------------------------------------------
// General private attributes.
//-------------------------------------------------------------------
    uint32_t ID;
    cXml *pXml;
    struct ObjectPoolEntry *pObjectPool;
    uint32_t ObjectPoolSize;
    uint32_t ObjectPoolIndex;
    bool bSystemExists;
//-------------------------------------------------------------------
// General private methods.
//-------------------------------------------------------------------
    int32_t StringCompare(const char *pString1, const char *pString2);
    bool bIsWhiteSpace(char c);
    uint32_t WhiteSpaceCount(const char *pString);
    bool bIsTokenDelimiter(char c);
    bool bIsDigit(char c);
    bool bIsInteger(const char *pString);
    bool bIsFloat(const char *pString);
    int64_t StringToInt64(const char *pString);
    double StringToDouble(const char *pString);
    float StringToFloat(const char *pString);
    int8_t StringToInt8(const char *pString);
    uint8_t StringToUint8(const char *pString);
    int16_t StringToInt16(const char *pString);
    uint16_t StringToUint16(const char *pString);
    int32_t StringToInt32(const char *pString);
    uint32_t StringToUint32(const char *pString);
    void Cleanup(void);
//-------------------------------------------------------------------
// Private factory methods.
//-------------------------------------------------------------------
    virtual void Factory(void) = 0;
    virtual void Scrap(void) = 0;

  public:
//-------------------------------------------------------------------
// General public methods.
//-------------------------------------------------------------------
    cSystem(uint32_t InstanceID);
    ~cSystem();
    uint32_t GetID(void);
    bool BuildSystem(const char *FileName);
    bool DestroySystem(void);
    void ParseElement(struct SimpleElement *pElement, uint32_t Length, bool bShowWarnings = false);
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
