//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 20.2.2009
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/hobbit_platform/cXml.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// The class 'cXmlException'.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The constructor of the class 'cXmlException' has these arguments:
// (1) The ID of the instance that has generated the exception.
// (2) A string with the name of the method that wants to report an
//     error, format 'ClassName::MethodName()'.
// (3) A 'structured string' that describes the error, for example
//     'Error.Port.Open'.
// (4) The number of the last file line that was successfully parsed
//     or the byte position within the file if an invalid character
//     was found.
// (5) A pointer to the ASCII(Z) data that caused the problem.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cXmlException::cXmlException(uint32_t ID, const char *Method, const char *Error,
                             uint32_t LineNumber, const char *Source) :
               cException(ID, Method, Error)
{

// Locally store the file line number and source pointer.
  FileLineNumber = LineNumber;
  SourcePointer = Source;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// If the error to be reported has happened during the first pass of
// the parser, this method will provide the corresponding number
// (starting at '1') of the text line within the file.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cXmlException::GetFileLineNumber(void)
{
  return FileLineNumber;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This method provides a pointer to the ASCII data that caused the
// exception. In case no pointer is to be provided, this value will
// be NULL.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const char *cXmlException::GetSourcePointer(void)
{
  return SourcePointer;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Prints the contents of the exception's attributes to the screen.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXmlException::Print(void)
{
  printf("\n*** cXmlException ***\n");
  printf("%s [%u]: %s\n", MethodName.c_str(), InstanceID, ErrorName.c_str());
  printf("In line #%u: %s\n\n", FileLineNumber, SourcePointer);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// General private methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Performs a cleanup, i.e. deletes all the resources that might have
// been allocated before.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXml::Cleanup(void)
{

// Remove the tree buffer if it exists.
  if (pTreeBuffer != (uint32_t *)NULL)
  {
    delete [] pTreeBuffer;
    pTreeBuffer = (uint32_t *)NULL;
  }

// Remove the file buffer if it exists.
  if (pFileBuffer != (char *)NULL)
  {
    delete [] pFileBuffer;
    pFileBuffer = (char *)NULL;
  }
  bFileLoaded = false;

// Close the file if it's still open.
  if (pFile != (FILE *)NULL)
  {
    fclose(pFile);
    pFile = (FILE *)NULL;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// File-related private methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Reads one character from the file buffer. The value of the argu-
// ment <bIncrease> decides if the read index will be increased after
// reading (default) or not.
// Invoking this method when the read index is already at the end of
// the buffer will result in an exception.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
char cXml::ReadCharacter(bool bIncrease)
{

// Check if we've already reached the end of the file buffer.
  if (ReadIndex >= FileBufferSize)
    throw cXmlException(ID, "cXml::ReadCharacter()", "Error.Unexpected.EOF",
                        FileLineNumber, pSource);

// Read one character from the file buffer and conditionally increase the read index.
  if (bIncrease) return pFileBuffer[ReadIndex++];
  else return pFileBuffer[ReadIndex];
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// Parsing-related private methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Checks the validity of the characters in the XML-file and norma-
// lises line breaks. There are only three valid byte values lexically
// before a SPACE character (0x20): TAB (0x09), LF (0x0a) and CR (0x0d).
// After successful processing, the ASCII data in the file buffer will
// be terminated with a zero byte.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXml::CheckAndNormalise(void)
{

// Loop through all bytes of the XML file.
  uint8_t uc;
  ReadIndex = 0;
  WriteIndex = 0;
  while (ReadIndex < FileBufferSize)
  {
    uc = (uint8_t)pFileBuffer[ReadIndex++];

// There are only three valid characters lexically before SPACE (0x20).
    if (uc < 0x20)
    {
      if ((uc != '\t') && (uc != '\n') && (uc != '\r'))
        throw cXmlException(ID, "cXml::CheckAndNormalise()", "Error.Invalid.Character",
                            (ReadIndex - 1), &pFileBuffer[ReadIndex - 1]);

// Replace a single CR or the combination CRLF by a single LF.
      else if (uc == '\r')
      {
        uc = '\n';
        if (ReadIndex < FileBufferSize)
          if (pFileBuffer[ReadIndex] == '\n') ReadIndex++;
      }
    }
    pFileBuffer[WriteIndex++] = (char)uc;
  }

// Update the size of the loaded file content after processing.
  FileBufferSize = WriteIndex;

// Terminate the processed file content with a zero byte.
  pFileBuffer[WriteIndex] = 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns 'true' if the character <c> is a whitespace character,
// 'false' otherwise.
// Whitespaces are SPACE, TAB and LF characters.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cXml::bIsWhitespace(const char c)
{
  if ((c == ' ') || (c == '\t') || (c == '\n')) return true;
  else return false;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Consumes whitespaces and returns the number of whitespace characters.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cXml::ConsumeWhitespaces(void)
{
  uint32_t u = 0;
  char c;
  while (bIsWhitespace(c = ReadCharacter(false)))
  {
    ReadIndex++;
    u++;

// In case a new line is encountered, the file line counter has to be increased by one.
    if (c == '\n') FileLineNumber++;
  }

// Return the number of whitespaces.
  return u;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Compares the string referenced by <pString2> to the first part of
// the string <pString1>. The return value is '-1' if <pString1> lies
// lexically before <pString2>, '1' if <pString1> lies lexically after
// <pString2>, and '0' if both strings are equal.
// This method is intended for searching for a keyword (<pString2>)
// within a longer string (<pString1>).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int32_t cXml::SubstringCompare(const char *pString1, const char *pString2)
{

// Compare both strings.
  uint32_t u = 0;
  char c;
  while (1)
  {
    c = pString2[u];
    if (c == 0x00) break;
    else if (pString1[u] == c) u++;
    else if (pString1[u] < c) return -1;
    else return 1;
  }
  return 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the length of the string referenced by <pString>. The
// length is the number of characters before a 0x00 has been en-
// countered.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cXml::StringLength(const char *pString)
{

// Determine the length of the string.
  uint32_t u = 0;
  while (pString[u] != 0x00) u++;
  return u;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This method is to be called when the character '<' has been de-
// tected. It consumes the tag-specific initial part of the tag and
// returns the type of the tag.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cXml::GetTagType(void)
{
  static const char *Keys[4] = {"/", "?xml ", "!--", "![CDATA["};
  static int32_t TagType[5] = {XML_TAG_END,
                               XML_TAG_DECLARATION,
                               XML_TAG_COMMENT,
                               XML_TAG_CDATA,
                               XML_TAG_START};

// Consume the tag's '<'.
  ReadIndex++;

// Figure out what tag type it is.
  uint32_t u;
  for (u = 0; u < 4; u++)
    if (SubstringCompare(&pFileBuffer[ReadIndex], Keys[u]) == 0) break;

// For tags other than the start tag, the first part of the tag is consumed.
  if (u < 4) ReadIndex = ReadIndex + StringLength(Keys[u]);
  return TagType[u];
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns 'true' if the character <c> is a lower or upper case
// character, 'false' otherwise.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cXml::bIsAlpha(const char c)
{
  if ((c >= 'A') && (c <= 'Z')) return true;
  else if ((c >= 'a') && (c <= 'z')) return true;
  else return false;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns 'true' if the character <c> is a digit, 'false' otherwise.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cXml::bIsDigit(const char c)
{
  return ((c >= '0') && (c <= '9'));
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns 'true' if the character <c> is alphanumeric, 'false'
// otherwise.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cXml::bIsAlnum(const char c)
{
  return (bIsAlpha(c) || bIsDigit(c));
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Moves <Length> bytes pointed at by <pStart> to a place in
// <pFileBuffer> indexed by <WriteIndex>. In the course of doing so,
// <WriteIndex> is adjusted.
// The return value is a pointer to the first byte of the moved data
// block.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
char *cXml::MoveData(char *pStart, uint32_t Length)
{
  char *NewPointer = &pFileBuffer[WriteIndex];

  for (uint32_t u = 0; u < Length; u++)
    pFileBuffer[WriteIndex++] = pStart[u];

  return NewPointer;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Parses an attribute and stores the information about the start and
// the length of the attribute's name and value in the member attributes
// <pAttributeName>, <AttributeNameLength> and <pAttributeValue>,
// <AttributeValueLength>, respectively.
// Accepted syntax is name="value" or name='value'. Spaces between
// name and '=' or between quotes and '=' are allowed.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXml::AttributeHandler(void)
{

// Get the attribute name and check its validity.
  pAttributeName = pFileBuffer + ReadIndex;
  AttributeNameLength = ReadIndex;
  LastNameLineNumber = FileLineNumber;
  char c = ReadCharacter();
  if (!bIsAlpha(c) && (c != '_'))
    throw cXmlException(ID, "cXml::AttributeHandler()", "Error.Invalid.CharacterInName",
                        FileLineNumber, pAttributeName);
  while ((c = ReadCharacter()) != '=')
  {
    if (bIsWhitespace(c)) break;
    else if (!bIsAlnum(c) && (c != '_') && (c != '-') && (c != '.'))
      throw cXmlException(ID, "cXml::AttributeHandler()", "Error.Invalid.CharacterInName",
                          FileLineNumber, pAttributeName);
  }
  AttributeNameLength = ReadIndex - (AttributeNameLength + 1);
  if (bIsWhitespace(c))
  {
    (void)ConsumeWhitespaces();
    if ((c = ReadCharacter()) != '=')
      throw cXmlException(ID, "cXml::AttributeHandler()", "Error.Invalid.Syntax",
                          FileLineNumber, &pFileBuffer[ReadIndex - 1]);
  }

// Get the attribute value and check the validity of the quoting.
  (void)ConsumeWhitespaces();
  pSource = &pFileBuffer[ReadIndex];
  LastValueLineNumber = FileLineNumber;
  char QuoteType = ReadCharacter();
  if ((QuoteType != '"') && (QuoteType != '\''))
    throw cXmlException(ID, "cXml::AttributeHandler()", "Error.No.Quote",
                        FileLineNumber, &pFileBuffer[ReadIndex - 1]);
  pAttributeValue = pFileBuffer + ReadIndex;
  AttributeValueLength = ReadIndex;
  while (ReadCharacter() != QuoteType) {};
  AttributeValueLength = ReadIndex - (AttributeValueLength + 1);

// Store the attribute name and value.
  pFileBuffer[WriteIndex++] = TOKEN_ATTRIBUTE;
  pAttributeName = MoveData(pAttributeName, AttributeNameLength);
  pFileBuffer[WriteIndex++] = 0x00;
  pAttributeValue = MoveData(pAttributeValue, AttributeValueLength);
  pFileBuffer[WriteIndex] = 0x00;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Compares the two strings referenced by <pString1> and <pString2>.
// The return value is '-1' if <pString1> lies lexically before
// <pString2>, '1' if <pString1> lies lexically after <pString2>,
// and '0' if both strings are equal. Two strings can only be equal
// if they have the same length.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int32_t cXml::StringCompare(const char *pString1, const char *pString2)
{

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
// This method is called when the XML declaration is found. It parses
// the declaration and processes its attributes.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXml::DeclarationHandler(void)
{

// Consume any whitespaces after the '<?xml'.
  (void)ConsumeWhitespaces();

// There has to be the attribute 'version' with the value '1.0'.
  AttributeHandler();
  if (StringCompare(pAttributeName, "version") != 0)
    throw cXmlException(ID, "cXml::DeclarationHandler()", "Error.Unexpected.Attribute",
                        LastNameLineNumber, pAttributeName);
  if (StringCompare(pAttributeValue, "1.0") != 0)
    throw cXmlException(ID, "cXml::DeclarationHandler()", "Error.Unsupported.Version",
                        LastValueLineNumber, pAttributeValue);

// There has to be at least one whitespace character.
  if (ConsumeWhitespaces() == 0)
    throw cXmlException(ID, "cXml::DeclarationHandler()", "Error.Missing.Whitespace",
                        FileLineNumber, &pFileBuffer[ReadIndex]);

// There has to be the attribute 'encoding' with the value 'UTF-8'.
  AttributeHandler();
  if (StringCompare(pAttributeName, "encoding") != 0)
    throw cXmlException(ID, "cXml::DeclarationHandler()", "Error.Unexpected.Attribute",
                        LastNameLineNumber, pAttributeName);
  if (StringCompare(pAttributeValue, "UTF-8") != 0)
    throw cXmlException(ID, "cXml::DeclarationHandler()", "Error.Unsupported.Encoding",
                        LastValueLineNumber, pAttributeValue);

// There has to be at least one whitespace character.
  if (ConsumeWhitespaces() == 0)
    throw cXmlException(ID, "cXml::DeclarationHandler()", "Error.Missing.Whitespace",
                        FileLineNumber, &pFileBuffer[ReadIndex]);

// There has to be the attribute 'standalone' with the value 'yes'.
  AttributeHandler();
  if (StringCompare(pAttributeName, "standalone") != 0)
    throw cXmlException(ID, "cXml::DeclarationHandler()", "Error.Unexpected.Attribute",
                        LastNameLineNumber, pAttributeName);
  if (StringCompare(pAttributeValue, "yes") != 0)
    throw cXmlException(ID, "cXml::DeclarationHandler()", "Error.Unsupported.Standalone",
                        LastValueLineNumber, pAttributeValue);
  (void)ConsumeWhitespaces();

// Consume the trailing '?>' of the declaration tag.
  if (SubstringCompare (&pFileBuffer[ReadIndex], "?>") != 0)
    throw cXmlException(ID, "cXml::DeclarationHandler()", "Error.Invalid.Syntax",
                        FileLineNumber, &pFileBuffer[ReadIndex]);
  ReadIndex = ReadIndex + 2;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This method is called whenever an end tag is encountered. It checks
// if the end tag is valid, that is, if there has been a correspon-
// ding start tag.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXml::EndTagHandler(void)
{
  pSource = &pFileBuffer[ReadIndex - 2];
  LastNameLineNumber = FileLineNumber;

// Check if we are already at the top hierarchy.
  if (HierarchyLevel == 0)
    throw cXmlException(ID, "cXml::EndTagHandler()", "Error.Already.TopOfHierarchy",
                        LastNameLineNumber, pSource);

  uint32_t u = WriteIndex;
  uint32_t v = HierarchyLevel;

// Search for the end of the tag name.
  pEndTagName = pFileBuffer + ReadIndex;
  EndTagNameLength = ReadIndex;
  while (ReadCharacter() != '>') {};

// Store the name of the end tag.
  pFileBuffer[WriteIndex++] = TOKEN_END_TAG;
  EndTagNameLength = ReadIndex - (EndTagNameLength + 1);
  pEndTagName = MoveData(pEndTagName, EndTagNameLength);
  pFileBuffer[WriteIndex] = 0x00;

// Search for the corresponding start tag.
  while (1)
  {
    if (u == 0)
      throw cXmlException(ID, "cXml::EndTagHandler()", "Error.No.MatchingStartTag",
                          LastNameLineNumber, pEndTagName);
    u--;
    if (pFileBuffer[u] == TOKEN_START_TAG)
    {
      if (v == HierarchyLevel)
      {
        char *pAux = pFileBuffer + (u + 1);
        u = 0;
        while (!((pAux[u] < 0x09) && (pEndTagName[u] < 0x09)))
        {
          if (pAux[u] != pEndTagName[u])
            throw cXmlException(ID, "cXml::EndTagHandler()", "Error.No.MatchingStartTag",
                                LastNameLineNumber, pEndTagName);
          else u++;
        }
        break;
      }
      else v--;
    }
    else if (pFileBuffer[u] == TOKEN_END_TAG) v++;
  }

// Decrease the hierarchy level by one.
  HierarchyLevel--;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This method is called whenever a comment is found. This comment is
// consumed. Comments may also reach over more than one text line.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXml::CommentHandler(void)
{
  pSource = &pFileBuffer[ReadIndex];

// A "state machine" is used to find the end of the comment.
  uint32_t State = 0;
  char c;
  while (State != 3)
  {
    c = ReadCharacter();

// In case a new line is encountered, the file line counter has to be increased by one.
    if (c == '\n') FileLineNumber++;

// Process the read character depending on the current state.
    else switch (State)
    {
      case 0:
        if (c == '-') State = 1;
        break;
      case 1:
        if (c == '-') State = 2;
        else State = 0;
        break;
      case 2:
        if (c == '>') State = 3;
        else if (c != '-') State = 0;
        break;
    }
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This method is called whenever a CDATA section is found.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXml::CDataHandler(void)
{
  pSource = &pFileBuffer[ReadIndex];

// Store preliminary information about the CDATA.
  CDataLength = ReadIndex;
  pCData = pFileBuffer + CDataLength;

// A "state machine" is used to find the end of the CDATA section.
  uint32_t State = 0;
  char c;
  while (State != 3)
  {
    c = ReadCharacter();

// In case a new line is encountered, the file line counter has to be increased by one.
    if (c == '\n') FileLineNumber++;

// Process the read character depending on the current state.
    else switch (State)
    {
      case 0:
        if (c == ']') State = 1;
        break;
      case 1:
        if (c == ']') State = 2;
        else State = 0;
        break;
      case 2:
        if (c == '>') State = 3;
        else if (c != ']') State = 0;
        break;
    }
  }

// Finalise the information about CDATA section.
  CDataLength = ReadIndex - (CDataLength + 3);
  pFileBuffer[WriteIndex++] = TOKEN_CDATA;
  pCData = MoveData(pCData, CDataLength);
  pFileBuffer[WriteIndex] = 0x00;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Checks the validity of the presumed start tag name including any
// attributes. Furthermore, a pointer to the name's first character
// as well as the length of the name are stored in the member
// attributes <pStartTagName> and <StartTagNameLength>.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXml::StartTagHandler(void)
{
  pSource = &pFileBuffer[ReadIndex];
  LastNameLineNumber = FileLineNumber;

// Find the end of the start tag, taking attributes' quotes into account.
  uint32_t u = ReadIndex;
  char c;
  char QuoteType = 0;
  while (1)
  {
    c = ReadCharacter();
    if (QuoteType != 0)
    {
      if (c == QuoteType) QuoteType = 0;
    }
    else if ((c == '"') || (c == '\'')) QuoteType = c;
    else if (c == '>') break;
  }

// Check if it is an empty tag.
  uint32_t v = ReadIndex;
  bool bIsEmpty = false;
  if (pFileBuffer[ReadIndex - 2] == '/') bIsEmpty = true;
  ReadIndex = u;

// Check the validity of the start tag name and search for its end.
  pStartTagName = pFileBuffer + ReadIndex;
  StartTagNameLength = ReadIndex;
  c = ReadCharacter();
  if (!bIsAlpha(c) && (c != '_'))
    throw cXmlException(ID, "cXml::StartTagHandler()", "Error.Invalid.StartTagName",
                        LastNameLineNumber, pSource);
  while (1)
  {
    c = ReadCharacter();
    if ((c == '>') || (c == '/') || bIsWhitespace(c)) break;
    else if (!bIsDigit(c) && !bIsAlpha(c) && (c != '_') && (c != '-') && (c != '.') && (c != ':'))
      throw cXmlException(ID, "cXml::StartTagHandler()", "Error.Invalid.StartTagName",
                          LastNameLineNumber, pSource);
  }

// Store the type and name of the start/empty tag.
  if (bIsEmpty) pFileBuffer[WriteIndex++] = TOKEN_EMPTY_TAG;
  else pFileBuffer[WriteIndex++] = TOKEN_START_TAG;
  StartTagNameLength = ReadIndex - (StartTagNameLength + 1);
  pStartTagName = MoveData(pStartTagName, StartTagNameLength);

// Take care of possible attributes.
  uint32_t w = 0;
  ReadIndex--;
  while (1)
  {
    u = ConsumeWhitespaces();
    c = ReadCharacter(false);
    if ((bIsEmpty && (c == '/')) || (!bIsEmpty && (c == '>')))
    {
      if ((c == '>') && (u != 0) && (w == 0))
        throw cXmlException(ID, "cXml::StartTagHandler()", "Error.Invalid.Whitespace",
                            LastNameLineNumber, pSource);
      break;
    }
    else if (u == 0)
      throw cXmlException(ID, "cXml::StartTagHandler()", "Error.No.Whitespace",
                          LastNameLineNumber, pSource);
    AttributeHandler();
    w++;
    AttributeCount++;
  }
  ReadIndex = v;

// In case it was not an empty tag, the hierarchy level is increased by one.
  if (!bIsEmpty)
  {
    HierarchyLevel++;
    if (MaxHierarchyLevel < HierarchyLevel) MaxHierarchyLevel = HierarchyLevel;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns 'true' if the character <c> is hexadecimal, 'false'
// otherwise.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cXml::bIsHex(const char c)
{
  if (bIsDigit(c)) return true;
  else if ((c >= 'A') && (c <= 'F')) return true;
  else if ((c >= 'a') && (c <= 'f')) return true;
  else return false;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the special character encoded by the entity reference. If
// no matching entity reference is found, an exception is thrown.
// Otherwise the referenced character is returned and the entity
// reference is consumed in the file buffer.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
char cXml::GetEntity(void)
{
  static const char *Keys[7] = {"amp;", "lt;", "gt;", "apos;", "quot;", "#x", "#"};
  static const char Characters[5] = {'&', '<', '>', '\'', '"'};

  char *pAscii = &pFileBuffer[ReadIndex - 1];
  LastValueLineNumber = FileLineNumber;

// Check if the entity reference is valid.
  uint32_t u;
  for (u = 0; u < 7; u++)
    if (SubstringCompare((pFileBuffer + ReadIndex), Keys[u]) == 0) break;
  if (u == 7)
    throw cXmlException(ID, "cXml::GetEntity()", "Error.Invalid.EntityReference",
                        LastValueLineNumber, pAscii);

// If so, adjust the file buffer read index.
  ReadIndex = ReadIndex + StringLength(Keys[u]);

// If it's one of the five basic entity references, return the respective entity.
  if (u < 5) return Characters[u];

// Otherwise, calculate the referenced decimal of hex value.
  else
  {
    uint32_t Temp = ReadIndex;
    uint16_t su = 0;
    char c;

// Calculate the hexadecimal's value and check its range.
    if (u == 5)
      while (bIsHex(c = ReadCharacter()))
      {
        if (bIsDigit(c)) c = c - 0x30;
        else c = (c & 0xdf) - 0x37;
        su = (su * 16) + (uint16_t)c;
        if (su > 255)
          throw cXmlException(ID, "cXml::GetEntity()", "Error.OutOfRange.Hexadecimal",
                              LastValueLineNumber, pAscii);
      }

// Calculate the decimal's value and check its range.
    else
      while (bIsDigit(c = ReadCharacter()))
      {
        c = c - 0x30;
        su = (su * 10) + (uint16_t)c;
        if (su > 255)
          throw cXmlException(ID, "cXml::GetEntity()", "Error.OutOfRange.Decimal",
                              LastValueLineNumber, pAscii);
      }

// Check if there has been at least one digit.
    if ((c != ';') || ((ReadIndex - Temp) == 1))
      throw cXmlException(ID, "cXml::GetEntity()", "Error.Invalid.EntityReference",
                          LastValueLineNumber, pAscii);
    return (char)su;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This method parses the content between tags. This PCDATA is ana-
// lysed for entity references that are NOT yet replaced by the re-
// spective character. Multiple text lines are allowed.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXml::PCDataHandler(void)
{
  pSource = &pFileBuffer[ReadIndex];

// PCDATA may only be encountered between tags.
  if (HierarchyLevel < 1)
    throw cXmlException(ID, "cXml::PCDataHandler()", "Error.Already.TopOfHierarchy",
                        FileLineNumber, pSource);
  pPCData = pFileBuffer + ReadIndex;
  pFileBuffer[WriteIndex++] = TOKEN_PCDATA;
  uint32_t u;
  char c;
  while (1)
  {
    c = ReadCharacter(false);

// The start of a tag ends the PCDATA section.
    if (c == '<') break;

// Check for invalid characters.
    else if ((c == '>') || (c == '"') || (c == '\''))
      throw cXmlException(ID, "cXml::PCDataHandler()", "Error.Invalid.Character",
                          FileLineNumber, &pFileBuffer[ReadIndex]);
    else
    {
      u = ReadIndex++;

// In case a new line is encountered, the file line counter has to be increased by one.
      if (c == '\n') FileLineNumber++;

// For the first parsing pass, entity references are checked but not yet resolved.
      else if (c == '&') (void)GetEntity();
      while (u < ReadIndex) pFileBuffer[WriteIndex++] = pFileBuffer[u++];
    }
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The first pass of the parser checks the syntax of the XML file,
// determines the required memory for generating a tree and allocates
// a buffer for the tree.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXml::PassOne(void)
{

// Allow (and throw away) leading empty text lines.
  ReadIndex = 0;
  WriteIndex = 0;
  (void)ConsumeWhitespaces();

// The first (code) line in an XML file must be the declaration.
  if (ReadCharacter(false) != '<')
    throw cXmlException(ID, "cXml::PassOne()", "Error.No.XmlDeclaration",
                        FileLineNumber, &pFileBuffer[ReadIndex]);
  if (GetTagType() != XML_TAG_DECLARATION)
    throw cXmlException(ID, "cXml::PassOne()", "Error.No.XmlDeclaration",
                        FileLineNumber, &pFileBuffer[ReadIndex - 1]);
  DeclarationHandler();
  (void)ConsumeWhitespaces();

// Throw away the declaration data.
  WriteIndex = 0;

// Initialise the statistics data before entering the first parsing pass.
  HierarchyLevel = 0;
  MaxHierarchyLevel = 0;
  NodeCount = 0;
  AttributeCount = 0;
  ContentCount = 0;

// Main parsing loop.
  uint32_t u;
  bool bRootEncountered = false;
  bool bWasContent = false;
  char c;
  while (!(bRootEncountered && (HierarchyLevel == 0)))
  {
    c = ReadCharacter(false);
    if (c == '<')
    {
      switch (GetTagType())
      {
        case XML_TAG_END:
          EndTagHandler();
          bWasContent = false;
          break;
        case XML_TAG_DECLARATION:
          throw cXmlException(ID, "cXml::PassOne()", "Error.Multiple.XmlDeclarations",
                              FileLineNumber, &pFileBuffer[ReadIndex]);
          break;
        case XML_TAG_COMMENT:
          CommentHandler();
          bWasContent = false;
          break;
        case XML_TAG_CDATA:
          if (!bWasContent) ContentCount++;
          CDataHandler();
          bWasContent = true;
          break;
        case XML_TAG_START:
          StartTagHandler();
          bRootEncountered = true;
          NodeCount++;
          bWasContent = false;
          break;
      }
    }
    else
    {
      u = ReadIndex;
      if (ConsumeWhitespaces() > 0)
        if (ReadCharacter(false) == '<') continue;
      ReadIndex = u;
      if (!bWasContent) ContentCount++;
      PCDataHandler();
      bWasContent = true;
    }
  }

  pFileBuffer[WriteIndex] = 0x00;
  for (u = 0; u < WriteIndex; u++)
    if (pFileBuffer[u] == 0x00) pFileBuffer[u] = 0x01;

// Space required for attribute name/value indices and content index/length pairs.
  TreeBufferSize = (AttributeCount + ContentCount) * 2;

// Space required for node information common to all nodes.
  TreeBufferSize = TreeBufferSize + (NodeCount * 6);

// Space required for storing the child node indices.
  TreeBufferSize = TreeBufferSize + (NodeCount - 1);

// Space required for storing the node list
  u = TreeBufferSize + NodeCount;

// Space required for storing the node hierarchy indices and the parsing stack.
  u = u + (2 * MaxHierarchyLevel);

// Allocate the memory for the tree buffer and associated data structures.
  pTreeBuffer = new (std::nothrow) uint32_t[u];
  if (pTreeBuffer == (uint32_t *)NULL)
    throw cSysCallException(ID, "cXml::PassOne()", "Error.SysCall.new", errno);
  pNodeList = pTreeBuffer + TreeBufferSize;
  pNodeHierarchy = pNodeList + NodeCount;
  pParsingStack = pNodeHierarchy + MaxHierarchyLevel;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Increases the read index (into the file buffer) until a byte with
// a value below 0x09 (TAB) is found. Such bytes are used as token
// delimiters by the parser.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXml::GoToNextToken(void)
{
  ReadIndex++;
  while (pFileBuffer[ReadIndex] >= 0x09) ReadIndex++;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the index of the next node in the tree buffer based on the
// index of the current node whose index is the argument of this
// method.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cXml::GetNextNode(uint32_t Node)
{

// Pointer to the attribute name/value indices.
  uint32_t NextNode = Node + 6;

// Pointer to the child node indices.
  NextNode = NextNode + (2 * pTreeBuffer[Node + 2]);

// Pointer to the content index/length pairs.
  NextNode = NextNode + pTreeBuffer[Node + 3];

// Pointer to the next node's data block.
  NextNode = NextNode + (2 * pTreeBuffer[Node + 4]);
  return NextNode;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The second pass of the parser generates the tree within the buffer
// previously allocated during pass one. For each start or empty tag
// a node is created.
// A node holds the following information:
// - index of the tag name (within the file buffer)
// - index of the parent node (within the tree buffer)
// - number of attributes
// - number of child nodes
// - number of contents
// - index of the next node within the same hierarchy level.
// - pairs of attribute name/value indices (within file buffer, optional)
// - indices to child nodes (within tree buffer, optional)
// - pairs of content index/length pairs (within file buffer, optional)
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXml::PassTwo(void)
{
  HierarchyLevel = MaxHierarchyLevel;
  ReadIndex = 0;
  CurrentNode = 0;
  uint32_t CurrentHierarchyLevel = MaxHierarchyLevel;
  uint32_t SaveReadIndex = ReadIndex;
  uint32_t u;
  bool bIsNode = false;
  char c;

// Zero-fill the tree buffer.
  for (u = 0; u < TreeBufferSize; u++) pTreeBuffer[u] = 0;

// Prepare the node hierarchy index list.
  for (u = 0; u < MaxHierarchyLevel; u++) pNodeHierarchy[u] = 0xffffffff;

// Loop until the end of the file buffer is encountered.
  while ((c = pFileBuffer[ReadIndex]) != 0x00)
  {

// For every start or empty tag, a node is created...
    if ((c == TOKEN_START_TAG) || (c == TOKEN_EMPTY_TAG))
    {

// ...but only if we're not searching for the child nodes...
      if (!bIsNode)
      {

// Store the file buffer index of the tag name and set the index to the next token.
        pTreeBuffer[CurrentNode] = ReadIndex;
        GoToNextToken();

// Update the node hierarchy index table.
        u = MaxHierarchyLevel - HierarchyLevel;
        pTreeBuffer[CurrentNode + 5] = pNodeHierarchy[u];
        pNodeHierarchy[u] = CurrentNode;

// Store the tree buffer index of the current node's parent node.
        if (HierarchyLevel == MaxHierarchyLevel) pTreeBuffer[CurrentNode + 1] = 0xffffffff;
        else
        {
          pTreeBuffer[CurrentNode + 1] = pParsingStack[HierarchyLevel];

// Store the current node's tree buffer index in its parent node's child node list.
          u = pParsingStack[HierarchyLevel];
          u = (2 * pTreeBuffer[u + 2]) + 6;
          u = u + pParsingStack[HierarchyLevel];
          while (pTreeBuffer[u] != 0) u++;
          pTreeBuffer[u] = CurrentNode;
        }

// Store the current node's attributes' name/value file buffer indices and adjust the attribute count.
        while (pFileBuffer[ReadIndex] == TOKEN_ATTRIBUTE)
        {
          u = CurrentNode + (2 * pTreeBuffer[CurrentNode + 2]) + 6;
          pTreeBuffer[u] = ReadIndex;
          GoToNextToken();
          pTreeBuffer[u + 1] = ReadIndex;
          GoToNextToken();
          pTreeBuffer[CurrentNode + 2]++;
        }

// If it's a start tag, it might have child nodes and content...
        SaveReadIndex = ReadIndex;
        if (c == TOKEN_START_TAG)
        {

// Store the current node's tree buffer index on the parsing stack and decrease the hieararchy level.
          HierarchyLevel--;
          CurrentHierarchyLevel = HierarchyLevel;
          pParsingStack[HierarchyLevel] = CurrentNode;
          bIsNode = true;
        }

// ...however, if it's an empty tag, we're done with this one.
        else CurrentNode = GetNextNode(CurrentNode);
      }

// Here we adjust the current node's child node count if it's a direct child node.
      else
      {
        if (CurrentHierarchyLevel == HierarchyLevel) pTreeBuffer[CurrentNode + 3]++;
        if (c == TOKEN_START_TAG) CurrentHierarchyLevel--;
        GoToNextToken();
      }
    }

// While looking for child nodes, attributes are just ignored.
    else if (c == TOKEN_ATTRIBUTE)
    {
      GoToNextToken();
      GoToNextToken();
    }

// Here we adjust the current node's content count if it's direct content.
    else if ((c == TOKEN_CDATA) || (c == TOKEN_PCDATA))
    {
      if (bIsNode & (CurrentHierarchyLevel == HierarchyLevel)) pTreeBuffer[CurrentNode + 4]++;
      while ((pFileBuffer[ReadIndex] == TOKEN_CDATA) || (pFileBuffer[ReadIndex] == TOKEN_PCDATA)) GoToNextToken();
    }

// Here we deal with an end tag.
    else if (c == TOKEN_END_TAG)
    {

// If there's no current node, only adjust the hierarchy level...
      if (!bIsNode)
      {
        GoToNextToken();
        HierarchyLevel++;
      }

// ...otherwise check if it's the current nodes or a child node's end tag.
      else
      {
        if (CurrentHierarchyLevel == HierarchyLevel)
        {
          ReadIndex = SaveReadIndex;
          CurrentNode = GetNextNode(CurrentNode);
          bIsNode = false;
        }
        else
        {
          GoToNextToken();
          CurrentHierarchyLevel++;
        }
      }
    }
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Convert a token in the file buffer into an ASCIIZ-string.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXml::MakeASCIIZ(void)
{

// Loop until the start of a new token is encountered.
  while (pFileBuffer[ReadIndex + 1] >= 0x09)
  {

// Remove the token ID.
    pFileBuffer[ReadIndex] = pFileBuffer[ReadIndex + 1];
    ReadIndex++;
  }

// Finalise the ASCII-string with a zero byte.
  pFileBuffer[ReadIndex++] = 0x00;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The third pass of the parser transforms tokens in the file buffer
// into ASCIIZ-strings and processes entity references in PCDATS
// sections. Furthermore, the content information stored in the
// nodes is finalised.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXml::PassThree(void)
{
  uint32_t u;
  char c;

  CurrentNode = 0;
  ReadIndex = 0;
  HierarchyLevel = MaxHierarchyLevel;

// Loop through all tokens in the file buffer.
  while ((c = pFileBuffer[ReadIndex]) != 0x00)
  {

// Transform start/empty tags and their attributes into ASCIIZ-strings.
    if ((c == TOKEN_START_TAG) || (c == TOKEN_EMPTY_TAG))
    {
      MakeASCIIZ();
      while (pFileBuffer[ReadIndex] == TOKEN_ATTRIBUTE) MakeASCIIZ();
      if (c == TOKEN_START_TAG)
        pParsingStack[--HierarchyLevel] = CurrentNode;
      CurrentNode = GetNextNode(CurrentNode);
    }

// End tags are just discarded.
    else if (c == TOKEN_END_TAG)
    {
      HierarchyLevel++;
      GoToNextToken();
    }

// Transform CDATA/PCDATA into ASCIIZ-strings.
    else if ((c == TOKEN_CDATA) || (c == TOKEN_PCDATA))
    {
      u = pParsingStack[HierarchyLevel] + 6;
      u = u + (2 * pTreeBuffer[pParsingStack[HierarchyLevel] + 2]);
      u = u + pTreeBuffer[pParsingStack[HierarchyLevel] + 3];
      while (pTreeBuffer[u] != 0) u = u + 2;
      pTreeBuffer[u] = ReadIndex;
      WriteIndex = ReadIndex;
      while (1)
      {
        c = pFileBuffer[ReadIndex];
        if (c == TOKEN_CDATA)
          while ((c = pFileBuffer[++ReadIndex]) >= 0x09) pFileBuffer[WriteIndex++] = c;
        else if (c == TOKEN_PCDATA)
        {
          ReadIndex++;
          while ((c = pFileBuffer[ReadIndex]) >= 0x09)
          {
            ReadIndex++;
            if (c == '&') c = GetEntity();
            pFileBuffer[WriteIndex++] = c;
          }
        }
        else break;
      }
      pFileBuffer[WriteIndex] = 0x00;
      pTreeBuffer[u + 1] = WriteIndex - pTreeBuffer[u];
    }
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// API-related private methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Adjusts the hierarchy level as well as the parsing stack after
// randomly selecting a node.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXml::AdjustParsingStack(void)
{
  uint32_t u, v, w, x;

// Determine the hierarchy level of the currently selected node.
  u = CurrentNode;
  HierarchyLevel = 0;
  while (pTreeBuffer[u + 1] != 0xffffffff)
  {
    u = pTreeBuffer[u + 1];
    HierarchyLevel++;
  }

// Start from the current node and at the current hierarchy level.
  u = CurrentNode;
  x = HierarchyLevel;

// Iterate through the current node's chain of parent nodes up to the root node.
  while (x > 0)
  {

// Determine the current node's index within its parent node's child node list.
    v = pTreeBuffer[u + 1] + 6 + (2 * pTreeBuffer[pTreeBuffer[u + 1] + 2]);
    w = 0;
    while (pTreeBuffer[v + w] != u) w++;

// Push this index onto the parsing stack and go to the next hierarchy level.
    x--;
    pParsingStack[x] = w;
    u = pTreeBuffer[u + 1];
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// General public methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The constructor of the class 'cXml' initialises the class' attri-
// butes and allocates the required resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cXml::cXml(uint32_t InstanceID)
{

// Locally store the ID of the instance of the class 'cXml'.
  ID = InstanceID;

// Initially there are no resources allocated.
  pFile = (FILE *)NULL;
  pFileBuffer = (char *)NULL;
  bFileLoaded = false;
  pTreeBuffer = (uint32_t *)NULL;
  NodeListLength = 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The destructor of the class 'cXml' frees the resources that have
// been allocated by the constructor.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cXml::~cXml()
{

// Just invoke the cleanup method.
  Cleanup();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the ID of this instance of the class 'cXml'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cXml::GetID(void)
{
  return ID;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// Public API methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Determines the size of the file, allocates a buffer and loads the
// file. The return value is 'true' in case there is no file loaded
// yet, 'false' otherwise.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cXml::LoadFile(const char *FileName)
{
  int i;

// Check if there is already a file loaded.
  if (bFileLoaded) return false;

// Determine the size of the file.
  if (stat(FileName, &(this->FileInfo)) == -1)
    throw cSysCallException(ID, "cXml::LoadFile()", "Error.SysCall.stat()", errno);
  FileBufferSize = (uint32_t)FileInfo.st_size;

// Allocate the file buffer with a size of (file size + 1).
  pFileBuffer = new (std::nothrow) char[FileBufferSize + 1];
  if (pFileBuffer == (char *)NULL)
    throw cSysCallException(ID, "cXml::LoadFile()", "Error.SysCall.new", errno);

// Load the file into the buffer.
  pFile = fopen(FileName, "rb");
  if (pFile == (FILE *)NULL)
  {
    i = errno;
    Cleanup();
    throw cSysCallException(ID, "cXml::LoadFile()", "Error.SysCall.fopen()", i);
  }
  if (fread((void *)pFileBuffer, 1, (size_t)FileBufferSize, pFile) != (size_t)FileBufferSize)
  {
    i = ferror(pFile);
    Cleanup();
    throw cSysCallException(ID, "cXml::LoadFile()", "Error.SysCall.fread()", i);
  }
  bFileLoaded = true;
  FileLineNumber = 1;

// Close the file.
  fclose(pFile);
  pFile = (FILE *)NULL;
  return true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Releases all resources allocated during loading the file and building
// the corresponding tree structure. In case there was no file loaded,
// the return value is 'false', 'true' otherwise.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cXml::UnloadFile(void)
{

// Check if a file has been loaded.
  if (!bFileLoaded) return false;
  Cleanup();
  return true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Parses the loaded file, determines what resources are required,
// builds a tree based on the Xml file and returns the number of
// tree nodes.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXml::BuildTree(void)
{

// Check if a file has been loaded.
  if (!bFileLoaded)
    throw cException(ID, "cXml::BuildTree()", "Error.No.File");

// Check if there is already a tree.
  if (pTreeBuffer != (uint32_t *)NULL)
    throw cException(ID, "cXml::BuildTree()", "Error.Already.Exists");

// Check the validity of the file's character data and normalise line breaks.
  CheckAndNormalise();

// Parse the XML file and build a tree.
  pSource = (char *)NULL;
  PassOne();
  PassTwo();
  PassThree();

// Go to the root node.
  GoToRootNode();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the current hierarchy level. The topmost level is '0',
// descending the tree increases the hierarchy level.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cXml::GetHierarchyLevel(void)
{
  return HierarchyLevel;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Makes the root node the current node.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cXml::GoToRootNode(void)
{
  CurrentNode = 0;
  HierarchyLevel = 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// If the currently selected node is not the root node, the node's
// parent node is made the new current node and 'true' is returned,
// otherwise only 'false' is returned.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cXml::GoToParentNode(void)
{

// Check if the current node is the root node.
  if (pTreeBuffer[CurrentNode + 1] == 0xffffffff) return false;

// If not, make the node's parent node the currently selected node...
  CurrentNode = pTreeBuffer[CurrentNode + 1];

// ...and adjust the hierarchy level.
  HierarchyLevel--;
  return true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the number of child nodes of the currently selected node.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cXml::GetNumberOfChildNodes(void)
{
  return pTreeBuffer[CurrentNode + 3];
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Selects the child node with the index <Child> as current node.
// Given a node has 'n' child nodes, allowed values for <Child> range
// from '0' to 'n - 1'. If <Child> lies within this interval, the
// specified child node is made the current node and 'true' is
// returned, otherwise only 'false' is returned.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cXml::GoToChildNode(uint32_t Child)
{

// Check if the specified child node exists.
  if (pTreeBuffer[CurrentNode + 3] <= Child) return false;

// If so, adjust the the parsing stack and hierarchy level...
  pParsingStack[HierarchyLevel] = Child;
  HierarchyLevel++;

// ...and make the child node the current node.
  Child = CurrentNode + Child + 6;
  Child = Child + (2 * pTreeBuffer[CurrentNode + 2]);
  CurrentNode = pTreeBuffer[Child];
  return true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Selects the sibling directly before the currently selected node.
// In case there is such a sibling, 'true' is returned, 'false'
// otherwise.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cXml::GoToPreviousSibling(void)
{

// Check if the currently selected node is already the first sibling or the root node.
  if (HierarchyLevel == 0) return false;
  else if (pParsingStack[HierarchyLevel - 1] == 0) return false;

// If not, get the index of the currently selected sibling from the parsing stack.
  uint32_t u = pParsingStack[HierarchyLevel - 1] - 1;

// Go to the parent node and then to the previous sibling.
  GoToParentNode();
  return GoToChildNode(u);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Selects the sibling directly after the currently selected node.
// In case there is such a sibling, 'true' is returned, 'false'
// otherwise.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cXml::GoToNextSibling(void)
{

// The root node has no siblings.
  if (HierarchyLevel == 0) return false;

// Get the index of the current sibling form the parsing stack.
  uint32_t u = pParsingStack[HierarchyLevel - 1] + 1;
  if (pTreeBuffer[pTreeBuffer[CurrentNode + 1] + 3] <= u) return false;

// Go to the parent node and then to the next sibling.
  GoToParentNode();
  return GoToChildNode(u);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Selects the first sibling of the currently selected node and returns
// the index of the sibling, which is always '0'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cXml::GoToFirstSibling(void)
{

// The root node is always the first sibling.
  if (HierarchyLevel == 0) return 0;

// Go to the parent node and then to its first child node.
  GoToParentNode();
  GoToChildNode(0);
  return 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Selects the last sibling of the currently selected node and returns
// the index of the sibling. If there are 'n' siblings, then the
// value returned is 'n - 1'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cXml::GoToLastSibling(void)
{

// The root node is always the last sibling.
  if (HierarchyLevel == 0) return 0;

// Go to the parent node and then to its last child node.
  GoToParentNode();
  uint32_t u = pTreeBuffer[CurrentNode + 3] - 1;
  GoToChildNode(u);
  return u;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the name of the currently selected node.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const char *cXml::GetNodeName(void)
{
  return &pFileBuffer[pTreeBuffer[CurrentNode]];
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the number of attributes of the currently selected node.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cXml::GetNumberOfAttributes(void)
{
  return pTreeBuffer[CurrentNode + 2];
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Checks if the currently selected node has an attribute with the
// specified name. If so, the number of the attribute is returned,
// '0xffffffff' otherwise.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cXml::HasAttributeWithName(const char *Name)
{
  uint32_t Index;

// Loop through all attributes of the currently selected node.
  for (uint32_t u = 0; u < GetNumberOfAttributes(); u++)
  {

// Calculate the index to the attribute name and compare the names.
   Index = (2 * u) + (CurrentNode + 6);
   if (StringCompare(&pFileBuffer[pTreeBuffer[Index]], Name) == 0) return u;
  }
  return 0xffffffff;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the name of the currently selcetd node's attribute with
// the number <Index>. If a node has 'n' attributes, the valid range
// for <Index> runs from '0' to 'n - 1'. In case an invalid attribute
// is specified, the NULL pointer is returned.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const char *cXml::GetAttributeName(uint32_t Index)
{
  if (pTreeBuffer[CurrentNode + 2] <= Index) return (char *)NULL;
  Index = (2 * Index) + (CurrentNode + 6);
  return &pFileBuffer[pTreeBuffer[Index]];
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the value of the currently selcetd node's attribute with
// the number <Index>. If a node has 'n' attributes, the valid range
// for <Index> runs from '0' to 'n - 1'. In case an invalid attribute
// is specified, the NULL pointer is returned.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const char *cXml::GetAttributeValue(uint32_t Index)
{
  if (pTreeBuffer[CurrentNode + 2] <= Index) return (char *)NULL;
  Index = (2 * Index) + (CurrentNode + 7);
  return &pFileBuffer[pTreeBuffer[Index]];
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the number of contents of the currently selected node.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cXml::GetNumberOfContents(void)
{
  return pTreeBuffer[CurrentNode + 4];
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the length of the currently selcetd node's content with
// the number <Index>. If a node has 'n' contents, the valid range
// for <Index> runs from '0' to 'n - 1'. In case an invalid content
// is specified, '0xffffffff' is returned.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cXml::GetContentLength(uint32_t Index)
{

// Check if the specified content exists.
  if (pTreeBuffer[CurrentNode + 4] <= Index) return 0xffffffff;

// If so, return its length.
  Index = 2 * (Index + pTreeBuffer[CurrentNode + 2]);
  Index = Index + pTreeBuffer[CurrentNode + 3];
  Index = Index + (CurrentNode + 7);
  return pTreeBuffer[Index];
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns a pointer to the currently selcetd node's content with
// the number <Index>. If a node has 'n' contents, the valid range
// for <Index> runs from '0' to 'n - 1'. In case an invalid content
// is specified, the NULL pointer is returned.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const char *cXml::GetContent(uint32_t Index)
{

// Check if the specified content exists.
  if (pTreeBuffer[CurrentNode + 4] <= Index) return (char *)NULL;

// If so, return a pointer to its data.
  Index = 2 * (Index + pTreeBuffer[CurrentNode + 2]);
  Index = Index + pTreeBuffer[CurrentNode + 3];
  Index = Index + (CurrentNode + 6);
  return &pFileBuffer[pTreeBuffer[Index]];
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Creates a list of all nodes with the specified tag name and with
// the specified attribute name. Also, the value of the attribute
// can be (optionally) specified. The return value is the number of
// nodes that fit the specifications.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cXml::GetNodesByNameWithAttribute(const char *NodeName, const char *AttributeName,
                                           const char *AttributeValue)
{

// Initially the node list is empty.
  NodeListLength = 0;

// Loop through all hierarchy levels.
  uint32_t v, w, x;
  for (uint32_t u = 0; u < MaxHierarchyLevel; u++)
  {

// Loop through all nodes within the current hierarchy level.
    v = pNodeHierarchy[u];
    while (v != 0xffffffff)
    {

// Check if the currently selected node's name is a match.
      if (NodeName != (const char *)NULL)
        if (StringCompare(NodeName, &pFileBuffer[pTreeBuffer[v]]) != 0)
          goto NoMatch;

// Check if an attribute name was specified.
      if (AttributeName != (const char *)NULL)
      {

// If so, check if there are any attributes.
        if (pTreeBuffer[v + 2] == 0) goto NoMatch;

// If so, check if there's a match with an attribute's name.
        x = v + 6;
        for (w = 0; w < pTreeBuffer[v + 2]; w++)
        {
          if (StringCompare(AttributeName, &pFileBuffer[pTreeBuffer[x]]) == 0)
          {

// If there's a match of the attribute name, check for a match of the attribute value.
            if (AttributeValue == (const char *)NULL) break;
            else if (StringCompare(AttributeValue, &pFileBuffer[pTreeBuffer[x + 1]]) != 0)
              goto NoMatch;
            else break;
          }
          x = x + 2;
        }
        if (w == pTreeBuffer[v + 2]) goto NoMatch;

      }

// Add the node to the node list.
      pNodeList[NodeListLength++] = v;

NoMatch:
      v = pTreeBuffer[v + 5];
    }
  }

// Restore the currently selected node and return the number of nodes.
  return NodeListLength;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Makes the node with the number <Index> of the node list the currently
// selected node and adjusts both, the parsing stack and the hierarchy
// level. Given there are 'n' nodes in the node list, valid values for
// <Index> range from '0' to 'n - 1'. The return value is 'true' if
// a valid node was specified, 'false' otherwise.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cXml::GoToNodeListNode(uint32_t Index)
{

// Check if the specified node list index is valid.
  if (NodeListLength <= Index) return false;

// Make the specified list node the current node.
  CurrentNode = pNodeList[Index];

// Adjust the content of the parsing stack.
  AdjustParsingStack();
  return true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
