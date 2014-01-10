//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 5.2.2009
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef C_XML_HH
#define C_XML_HH
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <new>
#include <string>
#include <inttypes.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "cException.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef NULL
#define NULL (void *)0
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// The class 'cXmlException'.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The class 'cXmlException' is derived from the base class 'cException'.
// It extends the base class with a UINT32 value and a pointer to a
// constant string. The first is intended to provide the file line
// number in which a parsing error occured and the latter will point
// to the ACSII data that caused the problem.
// In case the UINT32 value is zero, the first pass of the parser was
// successful and the error occured in a later stage - the value does
// no longer represent a file line number.
// The pointer to the ASCII data can be set to NULL if nothing is
// gained by providing the ASCII data.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cXmlException : public cException
{
  protected:
//-------------------------------------------------------------------
// Private attributes.
//-------------------------------------------------------------------
    uint32_t FileLineNumber;
    const char *SourcePointer;

  public:
//-------------------------------------------------------------------
// Public methods.
//-------------------------------------------------------------------
    cXmlException(uint32_t ID, const char *Method, const char *Error,
                  uint32_t LineNumber, const char *Source);
    uint32_t GetFileLineNumber(void);
    const char *GetSourcePointer(void);
    void Print(void);
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// The class 'cXml'.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Predefined XML tag types.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define XML_TAG_END                      0
#define XML_TAG_DECLARATION              1
#define XML_TAG_COMMENT                  2
#define XML_TAG_CDATA                    3
#define XML_TAG_START                    4
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Token types known to the parser.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define TOKEN_ATTRIBUTE                  1
#define TOKEN_EMPTY_TAG                  2
#define TOKEN_START_TAG                  3
#define TOKEN_END_TAG                    4
#define TOKEN_CDATA                      5
#define TOKEN_PCDATA                     6
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cXml
{
  protected:
//-------------------------------------------------------------------
// General private attributes.
//-------------------------------------------------------------------
    uint32_t ID;
//-------------------------------------------------------------------
// File-related private attributes.
//-------------------------------------------------------------------
    FILE *pFile;
    char *pFileBuffer;
    uint32_t FileBufferSize;
    struct stat FileInfo;
    bool bFileLoaded;
    uint32_t FileLineNumber;
//-------------------------------------------------------------------
// Parsing-related private attributes.
//-------------------------------------------------------------------
    uint32_t ReadIndex;
    uint32_t WriteIndex;
    uint32_t *pTreeBuffer;
    uint32_t NodeListLength;
    uint32_t HierarchyLevel;
    uint32_t MaxHierarchyLevel;
    uint32_t NodeCount;
    uint32_t AttributeCount;
    uint32_t ContentCount;
    char *pSource;
    uint32_t LastNameLineNumber;
    uint32_t LastValueLineNumber;
    char *pAttributeName;
    uint32_t AttributeNameLength;
    char *pAttributeValue;
    uint32_t AttributeValueLength;
    char *pEndTagName;
    uint32_t EndTagNameLength;
    char *pCData;
    uint32_t CDataLength;
    char *pStartTagName;
    uint32_t StartTagNameLength;
    char* pPCData;
    uint32_t TreeBufferSize;
    uint32_t *pParsingStack;
    uint32_t *pNodeHierarchy;
    uint32_t *pNodeList;
    uint32_t CurrentNode;
//-------------------------------------------------------------------
// File-related private methods.
//-------------------------------------------------------------------
    char ReadCharacter(bool bIncrease = true);
//-------------------------------------------------------------------
// Parsing-related private methods.
//-------------------------------------------------------------------
    void CheckAndNormalise(void);
    bool bIsWhitespace(const char c);
    uint32_t ConsumeWhitespaces(void);
    int32_t SubstringCompare(const char *pString1, const char *pString2);
    uint32_t StringLength(const char *pString);
    uint32_t GetTagType(void);
    bool bIsAlpha(const char c);
    bool bIsDigit(const char c);
    bool bIsAlnum(const char c);
    char *MoveData(char *pStart, uint32_t Length);
    void AttributeHandler(void);
    int32_t StringCompare(const char *pString1, const char *pString2);
    void DeclarationHandler(void);
    void EndTagHandler(void);
    void CommentHandler(void);
    void CDataHandler(void);
    void StartTagHandler(void);
    bool bIsHex(const char c);
    char GetEntity(void);
    void PCDataHandler(void);
    void PassOne(void);
    void GoToNextToken(void);
    uint32_t GetNextNode(uint32_t Node);
    void PassTwo(void);
    void MakeASCIIZ(void);
    void PassThree(void);
//-------------------------------------------------------------------
// API-related private methods.
//-------------------------------------------------------------------
    void AdjustParsingStack(void);
//-------------------------------------------------------------------
// General private methods.
//-------------------------------------------------------------------
    void Cleanup(void);

  public:
//-------------------------------------------------------------------
// General public methods.
//-------------------------------------------------------------------
    cXml(uint32_t InstanceID);
    ~cXml();
    uint32_t GetID(void);
//-------------------------------------------------------------------
// API-related public methods.
//-------------------------------------------------------------------
    bool LoadFile(const char *FileName);
    bool UnloadFile(void);
    void BuildTree(void);
    uint32_t GetHierarchyLevel(void);
    void GoToRootNode(void);
    bool GoToParentNode(void);
    uint32_t GetNumberOfChildNodes(void);
    bool GoToChildNode(uint32_t Child);
    bool GoToPreviousSibling(void);
    bool GoToNextSibling(void);
    uint32_t GoToFirstSibling(void);
    uint32_t GoToLastSibling(void);
    const char *GetNodeName(void);
    uint32_t GetNumberOfAttributes(void);
    uint32_t HasAttributeWithName(const char *Name);
    const char *GetAttributeName(uint32_t Index);
    const char *GetAttributeValue(uint32_t Index);
    uint32_t GetNumberOfContents(void);
    uint32_t GetContentLength(uint32_t Index);
    const char *GetContent(uint32_t Index);
    uint32_t GetNodesByNameWithAttribute(const char *NodeName,
                                         const char *AttributeName = (const char *)NULL,
                                         const char *AttributeValue = (const char *)NULL);
    bool GoToNodeListNode(uint32_t Index);
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
