ZASADY PRACY Z JIRA i REPOZYTORIUM:

**************************
1) COMMIT Z KOMENTARZEM :
************************** 

Gdy robimy commit z komentarzem commit automatycznie dodawany jest 
do wymienionego zadania w JIRA w komentarzu umieszczamy :


Składnia :
<ISSUE_KEY> <COMMAND> <COMMENT_STRING>

<ISSUE_KEY>  		oznaczecznie zadania z JIRA  : AUT-...
<COMMAND>    		słowo kluczowe : #commit 
<COMMENT_STRING>  	Krótki opis, zbyt długi zostanie ucięty, dłuższe opisy można robić powtarzając
			w każdej lini sekwencje <ISSUE_KEY> <COMMAND>

Przykład: 

AUT-125 #commit instrukcja tworzenia commitów 



###! Każda linia opisów w commicie musi zaczynać się <ISSUE_KEY> <COMMAND> by była zalogowana automatycznie w JIRA !###