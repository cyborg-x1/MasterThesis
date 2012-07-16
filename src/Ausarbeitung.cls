%%-*- LaTeX -*-%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Ausarbeitung.cls
%
% Klassendatei f�r Ausarbeitungen (Projektarbeiten, Bachelorarbeiten
% und Masterarbeiten.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Title:        ausarbeitung.cls\graphicspath{img}
% Author:       Ulrich Klauck und Manuel Blum (HTW Aalen)
% Modifiied by:
% Date:
% Last Rev.:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Identify the class.
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{Ausarbeitung}[2008/07/02 Ausarbeitung HTW Aalen]

% Default option---pass to scrbook.cls, since that is the base.
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{scrbook}}

% Do options.
\ProcessOptions

% Load the base class and required packages.
\LoadClassWithOptions{scrbook}
  \RequirePackage{pgf}
  \RequirePackage[english]{babel}
  \RequirePackage[right]{eurosym}
  \RequirePackage[automark]{scrpage2}
  \RequirePackage{times}
  \RequirePackage[T1]{fontenc}
  \RequirePackage[formats]{listings}
  \RequirePackage{algorithm}
  \RequirePackage{algpseudocode}
  \RequirePackage{struktex}
  \RequirePackage{calc}
  \RequirePackage{amsmath}
  \RequirePackage{array}
  \RequirePackage{graphicx}
  \RequirePackage{newcent}
  \RequirePackage{multicol}
  \RequirePackage{multirow}
  \RequirePackage{setspace}  
  \RequirePackage[utf8]{inputenc}
%  \RequirePackage[ansinew]{inputenc}
  \RequirePackage{placeins}
  \RequirePackage{caption}
  \RequirePackage{float}
%  \RequirePackage[final,colorlinks=false]{hyperref}

% Error information should be more than the default.
\setcounter{errorcontextlines}{5}
%\ClassError{Ausarbeitung}{Fehlertext}{Hilfe-Text}
%\ClassWarning{Ausarbeitung}{Warnung-Text}

%%%%%%%%%% Pagestyle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

\pagestyle{scrheadings} 
\clearscrheadfoot
\ihead{\headmark}
\ohead{\pagemark}
\raggedbottom

\onehalfspacing % Zeilenabstand: 1,5

%%%%%%%%%% Title %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

\def\MatrikelTok{\@empty}
\def\EMailTok{}
\def\BetreuerOneTok{\@empty} 
\def\BetreuerTwoTok{\@empty}
\def\AbgabeDatumTok{\@empty}
\def\FakultaetTok{\@empty}

\def\kandidat#1{\author{#1}}
\def\titel#1{\title{#1}}

\def\diplomarbeit{\subject{Diplomarbeit}}
\def\bachelorthesis{\subject{Bachelorthesis}}
\def\masterthesis{\subject{Master Thesis}}
\def\projektarbeit{\subject{Projektarbeit}}
\def\laborbericht{\subject{Laborbericht}}
\def\projektbericht{\subject{Projektbericht}}

\newboolean{mail}
\setboolean{mail}{false}
\newboolean{abgabe}
\setboolean{abgabe}{false}

\newcommand{\matrikelnr}[1]{
\def\MatrikelTok{#1}
}
\newcommand{\email}[1]{
\def\EMailTok{#1}
\setboolean{mail}{true}
}
\newcommand{\betreuer}[1]{
\def\BetreuerOneTok{#1}
}
\newcommand{\betreuertwo}[1]{
\def\BetreuerTwoTok{#1}
}
\newcommand{\abgabe}[1]{
\def\AbgabeDatumTok{#1}
\setboolean{abgabe}{true}
}
\newcommand{\fakultaet}[1]{
\def\FakultaetTok{#1}
}

\pgfdeclareimage[height=2.2cm]{htw-logo}{img/master-300-png}

\if@titlepage
  \renewcommand*\maketitle[1][1]{%
    \begin{titlepage}
      \setcounter{page}{#1}%
      \let\footnotesize\small
      \let\footnoterule\relax
      \let\footnote\thanks

      \renewcommand*\thefootnote{\@fnsymbol\c@footnote}%
      \let\@oldmakefnmark\@makefnmark
      \renewcommand*{\@makefnmark}{\rlap\@oldmakefnmark}%
      \ifx\@extratitle\@empty \else
        \noindent\@extratitle\next@tpage\cleardoublepage
        \thispagestyle{empty}%
      \fi
      
      
      %Kopfzeile%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      \begin{minipage}[l]{0.6\textwidth}
  		%\vspace{2px}
  		\fontsize{10}{0}\selectfont 
  		\FakultaetTok
  			\vspace{8px}
			\end{minipage}
			\begin{minipage}[r]{0.4\textwidth}
			 \begin{flushright}
  			\pgfuseimage{htw-logo}
  		 \end{flushright}
			\end{minipage}
			%			
      %Titel%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      \begin{center}
        %\vskip 5em
        \vskip 1cm
        \fontsize{14}{18}\selectfont
        \@subject\par
        %\vskip 2.5em  
        \vskip 0.5cm
        \fontsize{22}{28}\selectfont 
        \titlefont\@title\par        
        %\vskip 2.5em
        \vskip 1cm
        %
      %Autorblock%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
       %
       \fontsize{14}{18}\selectfont
       \normalfont %Kandidat: 
	\@author\par
			 %
       %Matrikel-Nr.: \MatrikelTok\par
       % 
       \ifthenelse{\boolean{mail}}
       	{E-Mail: \EMailTok}
       	{\vskip 0.4px}
       % 
       \vskip 1cm
       %
      %Abgabetermin%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
       \ifthenelse{\boolean{abgabe}}
       	{Closing Date: \AbgabeDatumTok}
       	{\vskip 21px}
       %
       \vskip 0.5cm
      %Betreuer%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
       Advisors:\par
         \BetreuerOneTok\par
         \BetreuerTwoTok
       	%
      \end{center}\par
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    \end{titlepage}
    \setcounter{footnote}{0}%
    \global\let\thanks\relax
    \global\let\maketitle\relax
    \global\let\@thanks\@empty
    \global\let\@author\@empty
    \global\let\@date\@empty
    \global\let\@title\@empty
    \global\let\@extratitle\@empty
    \global\let\@titlehead\@empty
    \global\let\@subject\@empty
    \global\let\@publishers\@empty
    \global\let\@uppertitleback\@empty
    \global\let\@lowertitleback\@empty
    \global\let\@dedication\@empty
    \global\let\author\relax
    \global\let\title\relax
    \global\let\extratitle\relax
    \global\let\titlehead\relax
    \global\let\subject\relax
    \global\let\publishers\relax
    \global\let\uppertitleback\relax
    \global\let\lowertitleback\relax
    \global\let\dedication\relax
    \global\let\date\relax
    \global\let\and\relax
  }
\else
  \renewcommand*\maketitle[1][1]{\par
    \@tempcnta=#1\relax\ifnum\@tempcnta=1\else
      \ClassWarning{\KOMAClassName}{%
        Optional argument of \string\maketitle\space ignored
        at\MessageBreak
        notitlepage-mode%
      }%
    \fi
    \begingroup
      \renewcommand*\thefootnote{\@fnsymbol\c@footnote}%
      \let\@oldmakefnmark\@makefnmark
      \renewcommand*{\@makefnmark}{\rlap\@oldmakefnmark}
      \if@twocolumn
        \ifnum \col@number=\@ne
          \@maketitle
        \else
          \twocolumn[\@maketitle]%
        \fi
      \else
        \newpage
        \global\@topnum\z@
        \@maketitle
      \fi
      \thispagestyle{\titlepagestyle}\@thanks
    \endgroup
    \setcounter{footnote}{0}%
    \let\thanks\relax
    \let\maketitle\relax
    \let\@maketitle\relax
    \global\let\@thanks\@empty
    \global\let\@author\@empty
    \global\let\@date\@empty
    \global\let\@title\@empty
    \global\let\@extratitle\@empty
    \global\let\@titlehead\@empty
    \global\let\@subject\@empty
    \global\let\@publishers\@empty
    \global\let\@uppertitleback\@empty
    \global\let\@lowertitleback\@empty
    \global\let\@dedication\@empty
    \global\let\author\relax
    \global\let\title\relax
    \global\let\extratitle\relax
    \global\let\titlehead\relax
    \global\let\subject\relax
    \global\let\publishers\relax
    \global\let\uppertitleback\relax
    \global\let\lowertitleback\relax
    \global\let\dedication\relax
    \global\let\date\relax
    \global\let\and\relax
  }
\fi
