#include "./rawlfsloader.h"
#include <string.h>
#include <stdio.h>
#include <cstddef>

namespace _BlackBill{
	namespace _Loader{

		const int BUFFER_SIZE = 1040;
		const int MAX_READ = 1024;

		const char TOKENNAME[][256] = {"TT_NONE", "TT_PROGSPEC", "TT_COMMENT", "TT_EQUAL", "TT_LABEL", "TT_BLANCKETBEGIN",	
			"TT_BLANCKETEND", "TT_STR", "TT_INT", "TT_FLOAT", "TT_ARRAYBEGIN", "TT_ARRAYEND", "TT_COMMA", "TT_DONTCARE"};

		const char STATEELEMENTNAME[][256] = {"SET_NONE", "SET_SPECSTR", "SET_LABEL", "SET_NUMARRAYSPEC", "SET_DATA"};

		const char STATEMENTNAME[][256] = {"ST_NONE", "ST_PROGSPEC", "ST_COMMENT", "ST_DATASPEC", "ST_DATAARRAYSPEC"};

		CRawLFSLoader::CRawLFSLoader()
			: m_FileName(NULL), m_FileBuffer(NULL), m_PrevChar(0), m_FileBufferSize(0), m_FileEnd(false), m_CharEnd(false), m_TokenError(false), m_TokenEnd(false), m_StatementError(false), m_Line(0), m_Verbose(true)
		{
			m_ProgramSpec.program = NULL;
			m_ProgramSpec.version = NULL;
		}

		CRawLFSLoader::~CRawLFSLoader()
		{
			if(NULL != m_FileName)
			{
				delete[] m_FileName;
				m_FileName = NULL;
			}

			if(NULL != m_FileBuffer)
			{
				delete[] m_FileBuffer;
				m_FileBuffer = NULL;
			}

			if(NULL != m_ProgramSpec.version)
			{
				delete[] m_ProgramSpec.version;
				m_ProgramSpec.version = NULL;
			}

			if(NULL != m_ProgramSpec.program)
			{
				delete[] m_ProgramSpec.program;
				m_ProgramSpec.program = NULL;
			}
		}

		bool CRawLFSLoader::loadFromFile(const char* in_FileName)
		{
			if(NULL != m_FileName)
				delete[] m_FileName;
			m_FileName = new char[strlen(in_FileName) + 1];
			strcpy(m_FileName, in_FileName);

			if(NULL == m_FileBuffer)
				m_FileBuffer = new char[BUFFER_SIZE];
			m_FileBufferPos = 0;

			m_FileEnd = false;
			m_CharEnd = false;
			m_TokenError = false;
			m_Line = 1;

			m_TokenPos = 0;
			m_PrevToken.type = TT_NONE;
			m_TokenEnd = false;
			m_StatementError = false;

			if((m_File = fopen(in_FileName, "rb")) == NULL)
				return false;
			if(!tokenize())
			{
				fclose(m_File);
				return false;
			}
			else if(!syntaxAnalysis())
			{
				fclose(m_File);
				return false;
			}
			else if(!semanticAnalysis())
			{
				printf("Semantic error.\n");
				fclose(m_File);
				return false;
			}
			else
			{
				fclose(m_File);
				return true;
			}
		}

		const map<string, SLFSDataField>* CRawLFSLoader::getConfigList()
		{
			return &m_DataFields;
		}

		void CRawLFSLoader::verbose(bool in_Switch)
		{
			m_Verbose = in_Switch;
		}

/**************************************************/
/*													Tokenize
/**************************************************/

		bool CRawLFSLoader::tokenize()
		{
			m_Tokens.clear();

			while((!m_CharEnd) && (!m_TokenError))
			{
				const char the_NextChar = getNextChar();

				if(the_NextChar == '=')
				{
					skipSpace();
					addToken(TT_EQUAL, "=");
				}
				else if(the_NextChar == ',')
				{
					skipSpace();
					addToken(TT_COMMA, ",");
				}
				else if(the_NextChar == '[')
				{
					skipSpace();
					addToken(TT_BLANCKETBEGIN, "[");
				}
				else if(the_NextChar == ']')
				{
					skipSpace();
					addToken(TT_BLANCKETEND, "]");
				}
				else if(the_NextChar == '{')
				{
					skipSpace();
					addToken(TT_ARRAYBEGIN, "{");
				}
				else if(the_NextChar == '}')
				{
					skipSpace();
					addToken(TT_ARRAYEND, "}");
				}
				else if(the_NextChar == '#')
				{
					readSpecialString();
				}
				else if(the_NextChar == '"')
					readString();
				else if((('0' <= the_NextChar) && (the_NextChar <= '9')) || (the_NextChar == '-') || (the_NextChar == '.'))
				{
					ungetNextChar(the_NextChar);
					readNumeric();
				}
				else
				{
					ungetNextChar(the_NextChar);
					readLabel();
				}
			}

			if(m_Verbose)
				showTokens();

			if(m_TokenError)
			{
				printf("Tokenize error: Line %d\n", m_Line);
				return false;
			}
			else
				return true;
		}

		char CRawLFSLoader::getNextChar()
		{
			if(m_PrevChar != 0)
			{
				const char ret = m_PrevChar;
				m_PrevChar = 0;
				return ret;
			}
			else
			{
				if(m_FileBufferPos >= m_FileBufferSize)
				{
					if(m_FileEnd)
					{
						m_CharEnd = true;
						return 0;
					}
					else
					{
						m_FileBufferSize = fread(m_FileBuffer, sizeof(char), MAX_READ, m_File);
						m_FileBufferPos = 0;

						if(feof(m_File))
							m_FileEnd = true;
					}
				}
				
				return m_FileBuffer[m_FileBufferPos++];
			}
		}

		void CRawLFSLoader::ungetNextChar(const char in_PrevChar)
		{
			m_PrevChar = in_PrevChar;
		}

		void CRawLFSLoader::skipSpace()
		{
			while(!m_CharEnd)
			{
				const char the_NextChar = getNextChar();
				if((the_NextChar != ' ') && (the_NextChar != '\t') && (the_NextChar != '\r') && (the_NextChar != '\n'))
				{
					ungetNextChar(the_NextChar);
					break;
				}

				if(the_NextChar == '\n')
					m_Line++;
			}
		}

		void CRawLFSLoader::readLine(char* io_Line, const int in_MaxChar)
		{
			int the_nRead = 0;
			while(!m_CharEnd)
			{
				const char the_NextChar = getNextChar();
				if(the_NextChar == '\n')
					break;

				if(the_NextChar == '\r')
					continue;

				if(the_nRead < in_MaxChar - 1)
					io_Line[the_nRead++] = the_NextChar;
			}

			io_Line[the_nRead] = 0;
			m_Line++;
		}

		void CRawLFSLoader::addToken(const ETokenType in_TokenType, const char* in_TokenStr)
		{
			SLFSToken the_Token;
			the_Token.type = in_TokenType;
			the_Token.token = in_TokenStr;
			m_Tokens.push_back(the_Token);
		}

		void CRawLFSLoader::showTokens()
		{
			for(int i=0; i<m_Tokens.size(); i++)
				printf("[%s] <%s>\n", TOKENNAME[m_Tokens[i].type], m_Tokens[i].token.data());
		}

		void CRawLFSLoader::showToken(const SLFSToken& in_Token)
		{
			printf("[%s] <%s>\n", TOKENNAME[in_Token.type], in_Token.token.data());
		}

		void CRawLFSLoader::readSpecialString()
		{
			const char the_NextChar = getNextChar();
			if(the_NextChar == '!')
			{
				char the_Line[BUFFER_SIZE];
				readLine(the_Line, MAX_READ);
				addToken(TT_COMMENT, the_Line);
			}
			else if(the_NextChar == '#')
			{
				const char the_NextNextChar = getNextChar();
				if(the_NextNextChar != '!')
					m_TokenError = true;
				else
				{
					char the_Line[BUFFER_SIZE];
					readLine(the_Line, MAX_READ);
					addToken(TT_PROGSPEC, the_Line);
				}
			}
			else
				m_TokenError = true;
		}

		void CRawLFSLoader::readString()
		{
			char the_Line[BUFFER_SIZE];
			bool the_FoundQuoteEnd = false;
			int the_nRead = 0;
			while(!m_CharEnd)
			{
				const char the_NextChar = getNextChar();
				if(the_NextChar == '"')
				{
					the_FoundQuoteEnd = true;
					break;
				}

				if(the_NextChar == '\n')
					m_Line++;

				if(the_nRead < MAX_READ - 1)
					the_Line[the_nRead++] = the_NextChar;
			}

			the_Line[the_nRead] = 0;
			if(!the_FoundQuoteEnd)
				m_TokenError = true;
			else
			{
				skipSpace();
				addToken(TT_STR, the_Line);
			}
		}

		void CRawLFSLoader::readNumeric()
		{
			char the_Line[BUFFER_SIZE];
			bool the_FoundPeriod = false;
			int the_nRead = 0;

			const char the_NextChar = getNextChar();
			the_Line[the_nRead++] = the_NextChar;

			while(!m_CharEnd)
			{
				const char the_NextChar = getNextChar();
				if(!((('0' <= the_NextChar) && (the_NextChar <= '9')) || (the_NextChar == '.')))
				{
					ungetNextChar(the_NextChar);
					break;
				}

				if(the_NextChar == '.')
				{
					if(the_FoundPeriod)
					{
						ungetNextChar(the_NextChar);
						break;
					}
					else
						the_FoundPeriod = true;
				}
								
				if(the_nRead < MAX_READ - 1)
					the_Line[the_nRead++] = the_NextChar;
			}

			the_Line[the_nRead] = 0;

			if((the_nRead == 1) && (the_Line[0] == '.'))
				m_TokenError = true;
			else
			{
				skipSpace();
				if(the_FoundPeriod)
					addToken(TT_FLOAT, the_Line);
				else
					addToken(TT_INT, the_Line);
			}
		}

		void CRawLFSLoader::readLabel()
		{
			char the_Line[BUFFER_SIZE];
			bool the_FoundPeriod = false;
			int the_nRead = 0;

			const char the_NextChar = getNextChar();
			if(!((the_NextChar == '_') || (('a' <= the_NextChar) && (the_NextChar <= 'z')) || (('A' <= the_NextChar) && (the_NextChar <= 'Z'))))
			{			
				m_TokenError = true;
				return;
			}

			the_Line[the_nRead++] = the_NextChar;

			while(!m_CharEnd)
			{
				const char the_NextChar = getNextChar();
				if(!((the_NextChar == '_') || (('a' <= the_NextChar) && (the_NextChar <= 'z')) || (('A' <= the_NextChar) && (the_NextChar <= 'Z')) || (('0' <= the_NextChar) && (the_NextChar <= '9'))))
				{
					ungetNextChar(the_NextChar);
					break;
				}
								
				if(the_nRead < MAX_READ - 1)
					the_Line[the_nRead++] = the_NextChar;
			}

			the_Line[the_nRead] = 0;
			skipSpace();
			addToken(TT_LABEL, the_Line);
		}

/**************************************************/
/*											SyntaxAnalysis
/**************************************************/

		bool CRawLFSLoader::syntaxAnalysis()
		{
			m_Statements.clear();

			while(1)
			{
				const SLFSToken the_NextToken = getNextToken();
				if(m_TokenEnd)
					break;

				if(the_NextToken.type == TT_PROGSPEC)
				{
					vector<SLFSStatementElement> the_StatementElements;
					SLFSStatementElement the_Element;
					the_Element.type = SET_SPECSTR;
					the_Element.tokenType = TT_PROGSPEC;
					the_Element.element = the_NextToken.token;
					the_StatementElements.push_back(the_Element);
					addStatement(ST_PROGSPEC, the_StatementElements);
				}
				else if(the_NextToken.type == TT_COMMENT)
				{
					vector<SLFSStatementElement> the_StatementElements;
					SLFSStatementElement the_Element;
					the_Element.type = SET_SPECSTR;
					the_Element.tokenType = TT_COMMENT;
					the_Element.element = the_NextToken.token;
					the_StatementElements.push_back(the_Element);
					addStatement(ST_COMMENT, the_StatementElements);
				}
				else if(the_NextToken.type == TT_LABEL)
				{
					ungetNextToken(the_NextToken);
					readStatementBeginWithLabel();
				}
				else
				{
					m_StatementError = true;
				}

				if(m_StatementError)
					break;
			}

			if(m_StatementError)
			{
				printf("Syntax error.\n");
				return false;
			}

			if(m_Verbose)
				showStatements();
			return true;
		}

		SLFSToken CRawLFSLoader::getNextToken()
		{
			if(m_PrevToken.type != TT_NONE)
			{
				SLFSToken ret = m_PrevToken;
				m_PrevToken.type = TT_NONE;
				return ret;
			}
			else
			{
				if(m_TokenPos >= m_Tokens.size())
				{
					m_TokenEnd = true;
					SLFSToken ret;
					ret.type = TT_NONE;
					return ret;
				}

				return m_Tokens[m_TokenPos++];
			}
		}

		void CRawLFSLoader::ungetNextToken(const SLFSToken& in_PrevToken)
		{
			m_PrevToken = in_PrevToken;
		}

		void CRawLFSLoader::showStatements()
		{
			for(int i=0; i<m_Statements.size(); i++)
			{
				printf("[[%s]]: ", STATEMENTNAME[m_Statements[i].type]);
				for(int j=0; j<m_Statements[i].statementElements.size(); j++)
				{
					printf("([%s]: ([%s] <%s>)) ", STATEELEMENTNAME[m_Statements[i].statementElements[j].type], TOKENNAME[m_Statements[i].statementElements[j].tokenType], m_Statements[i].statementElements[j].element.data());
				}
				printf("\n");
			}
		}

		void CRawLFSLoader::addStatement(const EStatementType in_StatementType, const vector<SLFSStatementElement>& in_StatementElements)
		{
			SLFSStatement the_Statement;
			the_Statement.type = in_StatementType;
			the_Statement.statementElements = in_StatementElements;
			m_Statements.push_back(the_Statement);
		}

		void CRawLFSLoader::readStatementBeginWithLabel()
		{
			const SLFSToken the_Label = getNextToken();
			const SLFSToken the_NextToken = getNextToken();

			vector<SLFSStatementElement> the_StatementElements;
			SLFSStatementElement the_Element;
			the_Element.type = SET_LABEL;
			the_Element.tokenType = TT_LABEL;
			the_Element.element = the_Label.token;
			the_StatementElements.push_back(the_Element);

			if(the_NextToken.type == TT_EQUAL)
			{
				const SLFSToken the_Data = getNextToken();
				if((the_Data.type != TT_STR) && (the_Data.type != TT_FLOAT) && (the_Data.type != TT_INT))
				{
					m_StatementError = true;
					return;
				}

				SLFSStatementElement the_Element;
				the_Element.type = SET_DATA;
				the_Element.tokenType = the_Data.type;
				the_Element.element = the_Data.token;
				the_StatementElements.push_back(the_Element);

				SLFSStatement the_Statement;
				the_Statement.type = ST_DATASPEC;
				the_Statement.statementElements = the_StatementElements;
				m_Statements.push_back(the_Statement);
			}
			else if(the_NextToken.type == TT_BLANCKETBEGIN)
			{
				ungetNextToken(the_NextToken);
				//[]�̓ǂݍ���
				if(!readSubStatementBlancket(&the_StatementElements))
				{
					m_StatementError = true;
					return;
				}
				//=�̓ǂݍ���
				const SLFSToken the_NextToken = getNextToken();
				if(the_NextToken.type != TT_EQUAL)
				{
					m_StatementError = true;
					return;
				}
				//{}�̓ǂݍ���
				if(!readSubStatementArray(&the_StatementElements))
				{
					m_StatementError = true;
					return;
				}
				
				SLFSStatement the_Statement;
				the_Statement.type = ST_DATAARRAYSPEC;
				the_Statement.statementElements = the_StatementElements;
				m_Statements.push_back(the_Statement);
			}
			else
			{
				m_StatementError = true;
			}
		}

		bool CRawLFSLoader::readSubStatementBlancket(vector<SLFSStatementElement>* io_StatementElements)
		{
			const SLFSToken the_BlancketStartToken = getNextToken();
			if(the_BlancketStartToken.type != TT_BLANCKETBEGIN)
				return false;

			const SLFSToken the_NextToken = getNextToken();

			if(the_NextToken.type == TT_BLANCKETEND)
			{
				SLFSStatementElement the_Element;
				the_Element.type = SET_NUMARRAYSPEC;
				the_Element.tokenType = TT_DONTCARE;
				the_Element.element = "";
				io_StatementElements->push_back(the_Element);
			}
			else if(the_NextToken.type == TT_INT)
			{
				const SLFSToken the_NextNextToken = getNextToken();
				if(the_NextNextToken.type != TT_BLANCKETEND)
					return false;
				SLFSStatementElement the_Element;
				the_Element.type = SET_NUMARRAYSPEC;
				the_Element.tokenType = the_NextToken.type;
				the_Element.element = the_NextToken.token;
				io_StatementElements->push_back(the_Element);
			}
			else
			{
				return false;
			}

			return true;
		}

		bool CRawLFSLoader::readSubStatementArray(vector<SLFSStatementElement>* io_StatementElements)
		{
			const SLFSToken the_ArrayStartToken = getNextToken();
			if(the_ArrayStartToken.type != TT_ARRAYBEGIN)
				return false;

			bool isPrevSeparated = true;

			while(1)
			{
				if(m_TokenEnd)
					return false;

				const SLFSToken the_NextToken = getNextToken();
				if(the_NextToken.type == TT_ARRAYEND)
					return true;
				else if((the_NextToken.type == TT_INT) || (the_NextToken.type == TT_FLOAT) || (the_NextToken.type == TT_STR))
				{
					if(!isPrevSeparated)
						return false;

					SLFSStatementElement the_Element;
					the_Element.type = SET_DATA;
					the_Element.tokenType = the_NextToken.type;
					the_Element.element = the_NextToken.token;
					io_StatementElements->push_back(the_Element);

                    isPrevSeparated = false;
				}
				else if(the_NextToken.type == TT_COMMA)
				{
					if(isPrevSeparated)
						return false;
					isPrevSeparated = true;
				}
				else
					return false;
			}
		}

/**************************************************/
/*										SemanticAnalysis
/**************************************************/

		bool CRawLFSLoader::semanticAnalysis()
		{
			bool isBody = false;
			if((m_Statements[0].type != ST_PROGSPEC) || (m_Statements[0].statementElements[0].element != "LFS"))
			{
				printf("Invalid Program Spec.\n");
				return false;
			}

			for(int i=1; i<m_Statements.size(); i++)
			{
				if(!isBody)
				{
					if((m_Statements[i].type == ST_DATASPEC) && (m_Statements[i].statementElements[0].element == "Version"))
					{
						if(NULL != m_ProgramSpec.version)
							delete[] m_ProgramSpec.version;
						m_ProgramSpec.version = new char[strlen(m_Statements[i].statementElements[1].element.data())+1];
						strcpy(m_ProgramSpec.version, m_Statements[i].statementElements[1].element.data());
					}
					else if((m_Statements[i].type == ST_PROGSPEC) && (m_Statements[i].statementElements[0].element == "Data"))
						isBody = true;
				}
				else
				{
					if((m_Statements[i].type == ST_DATASPEC))
					{
						string label = m_Statements[i].statementElements[0].element;
						SLFSDataFieldDataElement the_DataElement;
						if(m_Statements[i].statementElements[1].tokenType == TT_FLOAT)
						{
							the_DataElement.type = DT_FLOAT;
							the_DataElement.floatData = atof(m_Statements[i].statementElements[1].element.data());
						}
						else if(m_Statements[i].statementElements[1].tokenType == TT_INT)
						{
							the_DataElement.type = DT_INT;
							the_DataElement.intData = atoi(m_Statements[i].statementElements[1].element.data());
						}
						else if(m_Statements[i].statementElements[1].tokenType == TT_STR)
						{
							the_DataElement.type = DT_STR;
							the_DataElement.strData = new char[strlen(m_Statements[i].statementElements[1].element.data())+1];
							strcpy(the_DataElement.strData, m_Statements[i].statementElements[1].element.data());
						}
						else
						{
							return false;
						}

						SLFSDataField the_DataField;
						the_DataField.data.push_back(the_DataElement);
						the_DataField.isArray = false;
						the_DataField.isUsed = false;

						m_DataFields.insert(pair<string, SLFSDataField>(label, the_DataField));
					}
					else if(m_Statements[i].type == ST_DATAARRAYSPEC)
					{
						string label = m_Statements[i].statementElements[0].element;
						int expectedLen = 0;
						if(m_Statements[i].statementElements[1].type != SET_NUMARRAYSPEC)
							return false;
						if(m_Statements[i].statementElements[1].tokenType == TT_DONTCARE)
							expectedLen = -1;
						else if(m_Statements[i].statementElements[1].tokenType == TT_INT)
							expectedLen = atoi(m_Statements[i].statementElements[1].element.data());
						else
							return false;

						SLFSDataField the_DataField;
						the_DataField.isArray = true;
						the_DataField.isUsed = false;

						if((expectedLen >= 0) && (expectedLen != m_Statements[i].statementElements.size()-2))
						{
							printf("%d, %d\n", expectedLen, m_Statements[i].statementElements.size()-2);
							printf("Invalid array length.\n");
							return false;
						}

						for(int j=2; j<m_Statements[i].statementElements.size(); j++)
						{
							SLFSDataFieldDataElement the_DataElement;
							if(m_Statements[i].statementElements[j].tokenType == TT_FLOAT)
							{
								the_DataElement.type = DT_FLOAT;
								the_DataElement.floatData = atof(m_Statements[i].statementElements[j].element.data());
							}
							else if(m_Statements[i].statementElements[j].tokenType == TT_INT)
							{
								the_DataElement.type = DT_INT;
								the_DataElement.intData = atoi(m_Statements[i].statementElements[j].element.data());
							}
							else if(m_Statements[i].statementElements[j].tokenType == TT_STR)
							{
								the_DataElement.type = DT_STR;
								the_DataElement.strData = new char[strlen(m_Statements[i].statementElements[j].element.data())+1];
								strcpy(the_DataElement.strData, m_Statements[i].statementElements[j].element.data());
							}
							else
							{
								return false;
							}

							the_DataField.data.push_back(the_DataElement);
						}

						m_DataFields.insert(pair<string, SLFSDataField>(label, the_DataField));
					}
				}
			}

			if(m_Verbose)
				showDataFields();
			return true;
		}

		void CRawLFSLoader::showDataFields()
		{
			map<string, SLFSDataField>::iterator p = m_DataFields.begin();
			for(; p!=m_DataFields.end(); p++)
			{
				printf("[%s] (Used: ", p->first.data());
				if(p->second.isUsed)
					printf("true");
				else
					printf("false");
				printf(", Array: ");
				if(p->second.isArray)
				{
					printf("true, %d", p->second.data.size());
				}
				else
					printf("false");
				printf("): {");
				for(int i=0; i<p->second.data.size(); i++)
				{
					if(p->second.data[i].type == DT_INT)
						printf("[%d: INT]", p->second.data[i].intData);
					else if(p->second.data[i].type == DT_FLOAT)
						printf("[%f: FLOAT]", p->second.data[i].floatData);
					else if(p->second.data[i].type == DT_STR)
						printf("[%s: STR]", p->second.data[i].strData);

					if(i<p->second.data.size()-1)
						printf(", ");
				}
				printf("}\n");
			}
		}
	};
};
