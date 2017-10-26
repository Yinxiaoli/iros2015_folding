#ifndef __RAW_LFS_LOADER_H__
#define __RAW_LFS_LOADER_H__

#include <stdio.h>
#include <string>
#include <vector>
#include <map>
using namespace std;

namespace _BlackBill{
	namespace _Loader{

		enum ETokenType
		{
			TT_NONE,
			TT_PROGSPEC,					//##!
			TT_COMMENT,					//#!
			TT_EQUAL,							//=
			TT_LABEL,							//([A..Z]|[a..z]|[_]){([A..Z]|[a..z]|[_]|[0..9])}*
			TT_BLANCKETBEGIN,		//[																	//[] or [_]: dontcare, [n]: n
			TT_BLANCKETEND,			//]																	//[] or [_]: dontcare, [n]: n
			TT_STR,								//"..."
			TT_INT,									//
			TT_FLOAT,							//
			TT_ARRAYBEGIN,				//{
			TT_ARRAYEND,					//}
			TT_COMMA,
			TT_DONTCARE
		};

		enum EStatementElementType
		{
			SET_NONE,
			SET_SPECSTR,
			SET_LABEL,
			SET_NUMARRAYSPEC,
			SET_DATA
		};

		enum EStatementType
		{
			ST_NONE,
			ST_PROGSPEC,
			ST_COMMENT,
			ST_DATASPEC,
			ST_DATAARRAYSPEC
		};

		enum EDataType
		{
			DT_INT,
			DT_FLOAT,
			DT_STR
		};

		struct SLFSToken
		{
			ETokenType type;
			string token;
		};

		struct SLFSStatementElement
		{
			EStatementElementType type;
			ETokenType tokenType;
			string element;
		};

		struct SLFSStatement
		{
			EStatementType type;
			vector<SLFSStatementElement> statementElements;	//ÇÊÇËçÇãâÇ»åæåÍÇ…Ç∑ÇÈÇ…ÇÕÅAÇ±Ç±ÇÃïîï™Çì¸ÇÍéqç\ë¢Ç…Ç∑ÇÈ
		};

		struct SLFSProgramSpec
		{
			char* program;
			char* version;
		};

		struct SLFSDataFieldDataElement
		{
			EDataType type;
			union
			{
				int intData;
				float floatData;
				char* strData;
			};
		};

		struct SLFSDataField
		{
			bool isUsed;
			bool isArray;
			vector<SLFSDataFieldDataElement> data;
		};

		inline bool isDataNumeric(const SLFSDataFieldDataElement& in_Data)
		{
			return ((in_Data.type == DT_INT) || (in_Data.type == DT_FLOAT));
		}

		class CRawLFSLoader
		{
		public:
			CRawLFSLoader();
			~CRawLFSLoader();

			bool loadFromFile(const char* in_FileName);
			void verbose(bool in_Switch);
			const map<string, SLFSDataField>* getConfigList();
		protected:
			bool tokenize();
			bool syntaxAnalysis();
			bool semanticAnalysis();
			char getNextChar();
			void ungetNextChar(const char in_PrevChar);

			SLFSToken getNextToken();
			void ungetNextToken(const SLFSToken& in_PrevToken);

			void skipSpace();
			void addToken(const ETokenType in_TokenType, const char* in_TokenStr);
			void showTokens();
			void showToken(const SLFSToken& in_Token);
			void readLine(char* io_Line, const int in_MaxChar);

			void readSpecialString();
			void readString();
			void readNumeric();
			void readLabel();

			void addStatement(const EStatementType in_StatementType, const vector<SLFSStatementElement>& in_StatementElements);
			void showStatements();

			void readStatementBeginWithLabel();
			bool readSubStatementBlancket(vector<SLFSStatementElement>* io_StatementElements);
			bool readSubStatementArray(vector<SLFSStatementElement>* io_StatementElements);

			void showDataFields();

			char* m_FileName;
			FILE* m_File;
			char* m_FileBuffer;
			int m_FileBufferPos;
			int m_FileBufferSize;
			char m_PrevChar;
			bool m_FileEnd;
			bool m_CharEnd;

			int m_TokenPos;
			SLFSToken m_PrevToken;
			bool m_TokenEnd;
			
			bool m_StatementError;

			bool m_TokenError;
			int m_Line;
			vector<SLFSToken> m_Tokens;
			vector<SLFSStatement> m_Statements;

			SLFSProgramSpec m_ProgramSpec;
			map<string, SLFSDataField> m_DataFields;

			bool m_Verbose;
		};
	};
};

#endif
