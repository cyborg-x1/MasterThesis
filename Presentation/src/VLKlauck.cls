\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{VLKlauck}[2007/09/13 Vorlesungen Klauck HTW Aalen]

\DeclareOption*{%
  \PassOptionsToClass{\CurrentOption}{beamer}%
}
\ProcessOptions\relax
\ProcessOptions

\LoadClass{beamer}
 
\RequirePackage{pgf}
\RequirePackage{amsmath}
%\RequirePackage{amsfonts}
\newcommand{\field}[1]{\mathbb{#1}}
\newcommand{\C}{\field{C}}
\newcommand{\R}{\field{R}}

\RequirePackage{multicol}

%%\RequirePackage{tikz}
\RequirePackage[german]{babel}

\RequirePackage[utf8]{inputenc}

\RequirePackage{times}
\RequirePackage[T1]{fontenc}
\RequirePackage{graphicx}

%\RequirePackage{listing}
%\lstnewenvironment{c++}
%	{\lstset{language=c++}}
%	{}

\RequirePackage[formats]{listings}
\lstloadlanguages{[ISO]C++}
\lstdefineformat{C}{%
	\{=\newline\string\newline\indent,%
	\}=\newline\noindent\string\newline,%
	;=[\ ]\string\space}
\lstset{
   basicstyle=\small\ttfamily,                                  % print whole listing small
   keywordstyle=\color{black}\bfseries,     % underlined bold black keywords
   identifierstyle=,                                      % nothing happens
   commentstyle=\color{gray},                 % white comments
   stringstyle=\ttfamily,                              % typewriter type for strings
   showstringspaces=false,                      % no special string spaces
   tabsize=2}

%\RequirePackage{array}          % fuer aufwaendigere Tabellen
\RequirePackage{colortbl}       % farbige Tabellen (v. D. Carlisle)
%\RequirePackage{longtable}      % seitenuebergreifende Tabellen
%\RequirePackage{tabulary}

\RequirePackage{algorithm}
%%\RequirePackage{algpascal}
\RequirePackage{algpseudocode}
%%\renewcommand{\gets}{:=\ }
%%\renewcommand{$ \longleftarrow $ \ }
\RequirePackage{struktex}

\pgfdeclareimage[height=0.5cm]{htw-logo}{master-300-png}
\logo{\pgfuseimage{htw-logo}}

\mode<presentation>
{
  \useoutertheme{default}
  \usecolortheme{whale}
  \setbeamercovered{invisible}
}

\mode<handout> % 1 Folie auf einem Blatt, Seiten nummeriert
{
\setbeamertemplate{footline}
{
  \leavevmode%
  \hbox{%
  \begin{beamercolorbox}[wd=.1\paperwidth,ht=2.25ex,dp=1ex,left]{bg}%
    \usebeamerfont{pagenumber in head/foot}
    \insertpagenumber{} %%/ \inserttotalframenumber\hspace*{2ex} 
  \end{beamercolorbox}%
%
}%
  \vskip0pt%
}
}

\mode<handoutxx> % 4 Folien auf einem Blatt, Seiten nicht nummeriert
{
 \usepackage{pgfpages}
\pgfpagesuselayout{4 on 1}[a4paper,landscape,border shrink=5mm]
}

\mode<handoutxx> % 4 Folien auf einem Blatt, Seiten nummeriert
{
 \usepackage{pgfpages}
\pgfpagesuselayout{4 on 1}[a4paper,landscape,border shrink=5mm]
\setbeamertemplate{footline}
{
  \leavevmode%
  \hbox{%
  \begin{beamercolorbox}[wd=.1\paperwidth,ht=2.25ex,dp=1ex,left]{bg}%
    \usebeamerfont{pagenumber in head/foot}
    \insertpagenumber{} %%/ \inserttotalframenumber\hspace*{2ex} 
  \end{beamercolorbox}%
%
}%
  \vskip0pt%
}


}


\mode<handoutxx> % 2 Folien auf einem Blatt, Seite nummeriert
{
 \usepackage{pgfpages}
% \pgfpagesuselayout{2 on 1}[a4paper,landscape,border shrink=5mm, center=\pgfpoint{.5\pgfphysicalwidth}{.5\pgfphysicalheight}]

\pgfpagesuselayout{2 on 1}[a4paper,landscape,border shrink=5mm, center=\pgfpoint{.5\pgfphysicalwidth}{.5\pgfphysicalheight}]

\pgfpageslogicalpageoptions{1}
{%
resized width=.5\pgfphysicalwidth,%
resized height=.5\pgfphysicalheight,%
border shrink=\pgfpageoptionborder,%
center=\pgfpoint{.25\pgfphysicalwidth}{.75\pgfphysicalheight}%
}%
\pgfpageslogicalpageoptions{2}
{%
resized width=.5\pgfphysicalwidth,%
resized height=.5\pgfphysicalheight,%
border shrink=\pgfpageoptionborder,%
center=\pgfpoint{.75\pgfphysicalwidth}{.75\pgfphysicalheight}%
}% 

\setbeamertemplate{footline}
{
  \leavevmode%
  \hbox{%
  \begin{beamercolorbox}[wd=.1\paperwidth,ht=2.25ex,dp=1ex,left]{bg}%
    \usebeamerfont{pagenumber in head/foot}
    \insertpagenumber{} %%/ \inserttotalframenumber\hspace*{2ex} 
  \end{beamercolorbox}%
%
}%
  \vskip0pt%
}
}

\mode<handoutxx> % 2 Folien auf einem Blatt, Seite nicht nummeriert
{
 \usepackage{pgfpages}
% \pgfpagesuselayout{2 on 1}[a4paper,landscape,border shrink=5mm, center=\pgfpoint{.5\pgfphysicalwidth}{.5\pgfphysicalheight}]

\pgfpagesuselayout{2 on 1}[a4paper,landscape,border shrink=5mm, center=\pgfpoint{.5\pgfphysicalwidth}{.5\pgfphysicalheight}]

\pgfpageslogicalpageoptions{1}
{%
resized width=.5\pgfphysicalwidth,%
resized height=.5\pgfphysicalheight,%
border shrink=\pgfpageoptionborder,%
center=\pgfpoint{.25\pgfphysicalwidth}{.75\pgfphysicalheight}%
}%
\pgfpageslogicalpageoptions{2}
{%
resized width=.5\pgfphysicalwidth,%
resized height=.5\pgfphysicalheight,%
border shrink=\pgfpageoptionborder,%
center=\pgfpoint{.75\pgfphysicalwidth}{.75\pgfphysicalheight}%
}% 
}


\newcommand{\newframe}[1]
{
\begin{frame}[allowframebreaks,fragile]
#1
\end{frame}
}

\newcommand{\referenz}[1]
{
\begin{tiny}
#1
\end{tiny}
}

\renewcommand{\em}[1]{{\usebeamercolor[fg]{block title} #1}}

%\setlength{\parsep}{1ex}
\setlength{\parskip}{1ex}
\setcounter{tocdepth}{2}

\AtBeginSection[]
{
  \begin{frame}<beamer>[allowframebreaks]
    \frametitle{Inhalt}
    \tableofcontents[sectionstyle=show/show, subsectionstyle=show/show/show]
  \end{frame}
  \begin{frame}<handout>[allowframebreaks]
    \frametitle{Inhalt}
    \tableofcontents[sectionstyle=show/show, subsectionstyle=show/show/show]
  \end{frame}
}


\AtBeginSubsection[]
{
%  \begin{frame}<beamer>[allowframebreaks]
%    \frametitle{Inhalt}
%    \tableofcontents[sectionstyle=hide/hide, subsectionstyle= show/show/show]
%  \end{frame}
  \begin{frame}<beamer>[allowframebreaks]
    \frametitle{Inhalt}
    %\tableofcontents[sectionstyle=show/show, subsectionstyle= show/show/show]
    \tableofcontents[currentsubsection]
   \end{frame}
  \begin{frame}<handout>[allowframebreaks]
    \frametitle{Inhalt}
    %\tableofcontents[sectionstyle=show/show, subsectionstyle= show/show/show]
    \tableofcontents[currentsubsection]
   \end{frame}
}




