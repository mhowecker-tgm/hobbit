//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 24.3.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/MobilePlatform/cSystem.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// The class 'cSystemException'.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The constructor of the class 'cSystemException' has these arguments:
// (1) The ID of the instance that has generated the exception.
// (2) A string with the name of the method that wants to report an
//     error, format 'ClassName::MethodName()'.
// (3) A 'structured string' that describes the error, for example
//     'Error.Port.Open'.
// (4) The name of the error specific to this type of exception.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cSystemException::cSystemException(uint32_t ID, const char *Method, const char *Error, const char *SystemError) :
                  cException(ID, Method, Error)
{

// Locally store the name of the system error.
  SystemErrorName = SystemError;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the name of the system error'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const char *cSystemException::GetSystemError(void)
{
  return SystemErrorName.c_str();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Prints the contents of the exception's attributes to the screen.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cSystemException::Print(void)
{
  printf("\n*** cSystemException ***\n");
  printf("%s [%u]: %s\n", MethodName.c_str(), InstanceID, ErrorName.c_str());
  printf("%s\n\n", SystemErrorName.c_str());
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// General private methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Compares the two strings referenced by <pString1> and <pString2>.
// The return value is '-1' if <pString1> lies lexically before
// <pString2>, '1' if <pString1> lies lexically after <pString2>,
// and '0' if both strings are equal. Two strings can only be equal
// if they have the same length.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int32_t cSystem::StringCompare(const char *pString1, const char *pString2)
{

// Check the validity of the pointers to the input strings.
  if ((pString1 == 0) || (pString2 == 0))
    throw cException(ID, "cSystem::StringCompare()", "Error.Zero.Pointer");

// Compare both strings...
  uint32_t u = 0;
  char c;
  while (1)
  {
    c = pString2[u];
    if (pString1[u] == c)
    {

// ...taking into account that both must have the same length.
      if (c == 0x00) break;
      else u++;
    }
    else if (pString1[u] < c) return -1;
    else return 1;
  }
  return 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns 'true' if the character 'c' is a white space character,
// 'false' otherwise.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cSystem::bIsWhiteSpace(char c)
{
  if ((c == ' ') || (c == '\t')) return true;
  else return false;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the number of consecutive white space characters in the
// string referenced by 'pString'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cSystem::WhiteSpaceCount(const char *pString)
{
  uint32_t u;

// Check the validity of the pointer to the string.
  if (pString == 0)
    throw cException(0, "WhiteSpaceCount()", "Error.Zero.Pointer");

// Get the number of white space characters.
  u = 0;
  while (bIsWhiteSpace(pString[u])) u++;
  return u;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns 'true' if the character 'c' is a delimiter of a token,
// 'false' otherwise.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cSystem::bIsTokenDelimiter(char c)
{
  if ((c == '\0') || (c == '\r') || (c == '\n')) return true;
  else if ((c == ' ') || (c == '\t')) return true;
  else return false;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns 'true' if the character 'c' is a digit, 'false' otherwise.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cSystem::bIsDigit(char c)
{
  if ((c >= '0') && (c <= '9')) return true;
  else return false;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Checks if the token that is the argument of the method represents
// a valid integer number in ASCII format. If so, the return value is
// 'true', otherwise 'false'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cSystem::bIsInteger(const char *pString)
{

// Check if the token represents an integer number.
  bool bWasInt = false;
  uint32_t u = 0;
  char c;
  while (!bIsTokenDelimiter(c = pString[u]))
  {
    if ((c == '-') || (c == '+'))
    {
      if (u != 0) return false;
    }
    else if (bIsDigit(c)) bWasInt = true;
    else return false;
    u++;
  }
  return bWasInt;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Checks if the token that is the argument of the method represents
// a valid float number in ASCII format. If so, the return value is
// 'true', otherwise 'false'.
// Note that using an exponent is not (yet) supported. The decimal
// point is optional.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cSystem::bIsFloat(const char *pString)
{

// Check if the token represents a floating point number.
  bool bWasDot = false;
  bool bWasInt = false;
  uint32_t u = 0;
  char c;
  while (!bIsTokenDelimiter(c = pString[u]))
  {
    if ((c == '-') || (c == '+'))
    {
      if (u != 0) return false;
    }
    else if (c == '.')
    {
      if (bWasDot) return false;
      else bWasDot = true;
    }
    else if (bIsDigit(c)) bWasInt = true;
    else return false;
    u++;
  }
  return (bWasInt);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This method converts the ASCII representation of an integer, both
// signed and unsigned, into a 64bit integer. During conversion it
// checks if the boundaries of a 32bit integer have been violated.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int64_t cSystem::StringToInt64(const char *pString)
{

// Consume leading white spaces.
  const char *pAux = pString + WhiteSpaceCount(pString);

// Check and convert the string.
  if (!bIsInteger(pAux))
    throw cSystemException(ID, "cSystem::StringToInt64()", "Error.Not.Integer", pAux);

// Check if it's a positive or negative integer.
  bool bIsNegative = false;
  if (!bIsDigit(*pAux))
    if (*(pAux++) == '-') bIsNegative = true;

// Loop through all digits.
  int64_t lli = 0;
  int64_t Temp;
  while (!bIsTokenDelimiter(*pAux))
  {
    Temp = (int64_t)(*(pAux++) - '0');
    lli = (lli * (int64_t)10) + Temp;
    if (bIsNegative)
    {
      if (lli > (int64_t)2147483648)
        throw cException(ID, "cSystem::StringToInt64()", "Error.Value.Boundaries");
    }
    else if (lli > (int64_t)4294967295)
      throw cException(ID, "cSystem::StringToInt64()", "Error.Value.Boundaries");
  }

// Make sign adjustment and return.
  if (bIsNegative) lli = -lli;
  return lli;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the value of the previously extracted data or attribute
// as double value.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
double cSystem::StringToDouble(const char *pString)
{

// Convert string to double - TODO: check validity of string.
  return strtod(pString, 0);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the value of the previously extracted data or attribute
// as float value.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
float cSystem::StringToFloat(const char *pString)
{

// Convert string to float - TODO: check validity of string.
  return strtof(pString, 0);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Converts the ASCII string 'pString' into an int8 value.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int8_t cSystem::StringToInt8(const char *pString)
{

// Get the integer value.
  int64_t lli = StringToInt64(pString);

// Check the boundaries.
  if ((lli < (int64_t)(-128)) || (lli > (int64_t)127))
    throw cException(ID, "cSystem::StringToInt8()", "Error.Value.Boundaries");
  return (int8_t)lli;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Converts the ASCII string 'pString' into a uint8 value.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint8_t cSystem::StringToUint8(const char *pString)
{

// Get the integer value.
  int64_t lli = StringToInt64(pString);

// Check the boundaries.
  if ((lli < (int64_t)0) || (lli > (int64_t)255))
    throw cException(ID, "cSystem::StringToUint8()", "Error.Value.Boundaries");
  return (uint8_t)lli;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Converts the ASCII string 'pString' into an int16 value.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int16_t cSystem::StringToInt16(const char *pString)
{

// Get the integer value.
  int64_t lli = StringToInt64(pString);

// Check the boundaries.
  if ((lli < (int64_t)(-32768)) || (lli > (int64_t)32767))
    throw cException(ID, "cSystem::StringToInt16()", "Error.Value.Boundaries");
  return (int16_t)lli;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Converts the ASCII string 'pString' into a uint16 value.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint16_t cSystem::StringToUint16(const char *pString)
{

// Get the integer value.
  int64_t lli = StringToInt64(pString);

// Check the boundaries.
  if ((lli < (int64_t)0) || (lli > (int64_t)65535))
    throw cException(ID, "cSystem::StringToUint16()", "Error.Value.Boundaries");
  return (uint16_t)lli;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Converts the ASCII string 'pString' into an int32 value.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int32_t cSystem::StringToInt32(const char *pString)
{

// Get the integer value.
  int64_t lli = StringToInt64(pString);

// Check the boundaries.
  if (lli > (int64_t)2147483647)
    throw cException(ID, "cSystem::StringToInt32()", "Error.Value.Boundaries");
  return (int32_t)lli;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Converts the ASCII string 'pString' into a uint32 value.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cSystem::StringToUint32(const char *pString)
{

// Get the integer value.
  int64_t lli = StringToInt64(pString);

// Check the boundaries.
  if (lli < (int64_t)0)
    throw cException(ID, "cSystem::StringToUint32()", "Error.Value.Boundaries");
  return (uint32_t)lli;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Performs a cleanup, i.e. deletes all the resources that might have
// been allocated before.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cSystem::Cleanup(void)
{

// Conditionally remove the object pool.
  if (pObjectPool != 0)
  {
    delete [] pObjectPool;
    pObjectPool = 0;
  }

// Conditionally remove the instance of the class 'cXml'.
  if (pXml != 0)
  {
    pXml->UnloadFile();
    delete pXml;
    pXml = 0;
  }
  bSystemExists = false;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// General public methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The constructor of the class 'cSystem' initialises the class'
// attributes and allocates the required resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cSystem::cSystem(uint32_t InstanceID)
{

// Locally store the ID of the instance of the class 'cSystem'.
  ID = InstanceID;

// Initially there are no resources allocated.
  pXml = 0;
  pObjectPool = 0;
  ObjectPoolIndex = 0;
  bSystemExists = false;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The destructor of the class 'cSystem' frees the resources that have
// been allocated by the constructor.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cSystem::~cSystem()
{
  Cleanup();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the ID of this instance of the class 'cSystem'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cSystem::GetID(void)
{
  return ID;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cSystem::BuildSystem(const char *FileName)
{

// Check if there's already an existing system.
  if (bSystemExists) return false;

// Check the validity of the filename.
  if (FileName == 0)
    throw cException(ID, "cSystem::BuildSystem()", "Error.Zero.Pointer");

// Create an instance of the class 'cXml'.
  pXml = new (std::nothrow) cXml(ID);
  if (pXml == 0)
    throw cSysCallException(ID, "cSystem::BuildSystem()", "Error.SysCall.new", errno);

  try
  {
    pXml->LoadFile(FileName);
    pXml->BuildTree();
  }
  catch (cException e)
  {
    delete pXml;
    pXml = 0;
    throw;
  }

// Find all nodes with the name 'Object'.
  ObjectPoolSize = pXml->GetNodesByNameWithAttribute(0, "type", "object");
  printf(" cSystem::BuildSystem(): object pool size is %u\n", ObjectPoolSize);
  if (ObjectPoolSize > 0)
  {
    int i;
    pObjectPool = new (std::nothrow) struct ObjectPoolEntry[ObjectPoolSize];
    if (pObjectPool == 0)
    {
      i = errno;
      Cleanup();
      throw cSysCallException(ID, "cSystem::BuildSystem()", "Error.SysCall.new", i);
    }

    uint32_t u = ObjectPoolSize;
    while (u > 0)
    {
      u--;
      (void)pXml->GoToNodeListNode(u);
      Factory();
    }
  }

// Go to the root node.
  pXml->GoToRootNode();

// Now there's an existing system.
  bSystemExists = true;
  return true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// In case the system has been successfully built before, this method
// is used to release all allocated resources. First, all the objects
// in the object pool are destroyed. Then, the object pool is deleted,
// and finally, the currently active XML-file is released.
// Calling this method with a system existing, the return value is
// 'true', 'false' otherwise.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cSystem::DestroySystem(void)
{

// Only do something if there's an existing system.
  if (!bSystemExists) return false;

// Destroy all objects in the object pool.
  Scrap();

// Delete the object pool and release the active XML-file.
  Cleanup();
  return true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cSystem::ParseElement(struct SimpleElement *pElement, uint32_t Length, bool bShowWarnings)
{

// Check the validity of the arguments.
  if (pElement == 0)
    throw cException(ID, "cSystem::ParseElement()", "Error.Zero.Pointer");
  if (Length == 0)
    throw cException(ID, "cSystem::ParseElement()", "Error.Zero.Length");

  uint32_t u, v, w, x;
  const char *pAux;
  char *pSrc, *pDest;

// Loop through all the child nodes of the current node.
  for (u = 0; u < pXml->GetNumberOfChildNodes(); u++)
  {

// Go to a child node and determine its name.
    pXml->GoToChildNode(u);
    pAux = pXml->GetNodeName();

// Check if the node corresponds to an entry in the provided list.
    for (v = 0; v < Length; v++)
      if (!pElement[v].bInitialised)
        if (StringCompare(pElement[v].TagName, pAux) == 0) break;
    if (v == Length)
    {
      if (bShowWarnings)
        printf("*** WARNING: the node '%s' has no corresponding partner in the provided list!\n", pAux);
      pXml->GoToParentNode();
      continue;
    }

// Process the content of the node depending on the data type.
    if (pElement[v].DataType == ELEMENT_NODE)
      ParseElement((struct SimpleElement *)pElement[v].Attribute, pElement[v].Size);
    else if (pElement[v].DataType == ELEMENT_OBJECT)
    {
      pAux = pXml->GetAttributeValue(pXml->HasAttributeWithName("interface"));
      for (w = 0; w < ObjectPoolIndex; w++)
        if (pAux == pObjectPool[w].pInterface) break;
      if (w < ObjectPoolIndex)
        *((void **)pElement[v].Attribute) = pObjectPool[w].pObject;
      else throw cSystemException(ID, "cSystem::ParseElement()", "Error.No.Match", pAux);
    }
    else
    {
      pAux = pXml->GetContent(0);
      switch (pElement[v].DataType)
      {
        case ELEMENT_UINT8:
          *((uint8_t *)pElement[v].Attribute) = StringToUint8(pAux);
          break;
        case ELEMENT_INT8:
          *((int8_t *)pElement[v].Attribute) = StringToInt8(pAux);
          break;
        case ELEMENT_UINT16:
          *((uint16_t *)pElement[v].Attribute) = StringToUint16(pAux);
          break;
        case ELEMENT_INT16:
          *((int16_t *)pElement[v].Attribute) = StringToInt16(pAux);
          break;
        case ELEMENT_UINT32:
          *((uint32_t *)pElement[v].Attribute) = StringToUint32(pAux);
          break;
        case ELEMENT_INT32:
          *((int32_t *)pElement[v].Attribute) = StringToInt32(pAux);
          break;
        case ELEMENT_FLOAT:
          *((float *)pElement[v].Attribute) = StringToFloat(pAux);
          break;
        case ELEMENT_DOUBLE:
          *((double *)pElement[v].Attribute) = StringToDouble(pAux);
          break;
        case ELEMENT_STRING:
          pSrc = (char *)pAux;
          pDest = (char *)pElement[v].Attribute;
          w = 0;
          while (pAux[w] != '\0') w++;
          if (pElement[v].Size < w) w = pElement[v].Size;
          for (x = 0; x < w; x++) pDest[x] = pSrc[x];
          pDest[x] = '\0';
          break;
        case ELEMENT_BOOLEAN:
          if (StringCompare(pAux, "true") == 0) *((bool *)pElement[v].Attribute) = true;
          else if (StringCompare(pAux, "false") == 0) *((bool *)pElement[v].Attribute) = false;
          else throw cException(ID, "cSystem::ParseElement()", "Error.Invalid.Syntax");
          break;
        default:
         throw cException(ID, "cSystem::ParseElement()", "Error.Invalid.DataType");
          break;
      }
    }

// Mark the attribute as initialised and end and go to the parent node.
    pElement[v].bInitialised = true;
    pXml->GoToParentNode();
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
