from pprint import pprint
import re
import sys

class JSGFParser():

    def __init__(self, fileName=None):
        if fileName:
            self.fileName = fileName
            self.loadGrammarFile(self.fileName)

    def loadGrammarFile(self, fileName):
        with open(fileName, "r+") as grammarFile:
            content = grammarFile.read()
        with open(fileName, "r+") as grammarFile:
            lines = grammarFile.readlines()

        grammarFileLines = []
        for line in lines:
            line = line.strip()
            if line and "=" in line:
                grammarFileLines.append(line)

        self.grammarFileContent = content
        self.grammarFileLines = grammarFileLines

        self.tokens = self.getAllTokens()

    def findStopwords(self):
        """
            Find all optional tokens that has the tag 'stopword'
        """
        p = re.compile(ur'(?<=\[)[a-z< >]+(?=\] *{stopword})') # find iems within [] followed by {stopword}
        matches = re.findall(p, self.grammarFileContent)
        uniqueMatches = list(set(self.traverseList(matches)))

        return uniqueMatches

    def getAllTokens(self):
        # tokens = re.compile(ur'(?<=[|=()]) *[a-z ]+ *(?<tag>\{[a-z_]+\} *)?(?=[|;()])')
        p = re.compile(ur'(?<=[|=()]) *([a-z ]+) *(\{[a-z_]+\})? *(?=[|;()])')
        # p = re.compile(ur'(\<[a-z_]+\>).*(?<=[|=()]) *([a-z ]+) *(\{[a-z_]+\})? *(?=[|;()]).*')
        matches = re.findall(p, self.grammarFileContent)
        tokens = []
        for match in matches:
            token = match[0].strip()
            if token:
                tag = match[1].strip('{}')
                var = self.findTokenVar(token)
                tokens.append({"text": token, "tag": tag, "var": var})

        return tokens

    def findToken(self, string):
        for token in self.tokens:
            if token["text"] == string.strip():
                return token

    def findTokenVar(self, token):
        patternString = ur'(\<[a-z_]+\>).*(?:' + token + ')'
        p = re.compile(patternString)
        matches = re.findall(p, self.grammarFileContent)

        if matches:
            return matches[0]
        else:
            return None


    def findOptionalTokens(self):
        tokenList = []
        for line in self.grammarFileLines:
            tokens = self.findEnclosures(line)
            if tokens:
                for token in tokens:
                    token = token[1:-1]
                    tokenList.append(token)

        tokenList = list(set(tokenList))
        pprint(tokenList)
        tokenList = self.traverseList(tokenList)
        pprint(tokenList)

    def traverseList(self, list):
        newList = []
        for item in list:
            if self.isVariable(item):
                newList += self.traverseList(self.findVariableItems(item))
            else:
                newList.append(item)
        return newList

    def findVariableItems(self, item, includeVars = True):
        items = []
        for line in self.grammarFileLines:
            parts = line.split("=")
            if item in parts[0]:
                if includeVars:
                    p = re.compile(ur'(?<=[|=(]) *([a-z_<> ]+) *(\{[a-z_]+\})? *(?=[|;)])')
                else:
                    p = re.compile(ur'(?<=[|=(]) *([a-z_ ]+) *(\{[a-z_]+\})? *(?=[|;)])')                    
                matches = re.findall(p, line)
                for match in matches:
                    items.append(match[0].strip())

                # tokens = parts[1].split("|")
                # for token in tokens:
                #     items.append(token.strip().strip(";"))

        return items


    def isVariable(self, item):
        if item[0:1] == "<" and item[-1:] == ">":
            return True

        return False

    def findEnclosures(self, line):
        # m = re.findall(r"\[[^]^[]*\]|\([^)^(]*\)|\([^}^{]*\)", line)
        m = None
        m = re.findall(r"\[[^]^[]*\]", line)
        return m

if __name__ == '__main__':
    parser = JSGFParser("grammar/experiment.gram")

    # parser.findOptionalTokens()
    # parser.findStopwords()
    # parser.getAllTokens()
    tokenString = " "
    foundToken = parser.findToken(tokenString)
    pprint(foundToken)