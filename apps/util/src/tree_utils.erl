% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                                                         %
%  This file is created as part of research done by the Multi-Agent Coordination & Control (MACC) group.  %
%                                                                                                         %
%  (C) 2013 Johan Philips, Bart Saint Germain, Jan Van Belle, Paul Valckenaers, macc.radar@gmail.com      %
%                                                                                                         %
%  Department of Mechanical Engineering, Katholieke Universiteit Leuven, Belgium.                         %
%                                                                                                         %
%  You may redistribute this software and/or modify it under either the terms of the GNU Lesser           %
%  General Public License version 2.1 (LGPLv2.1 <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>) %
%  or (at your discretion) of the Modified BSD License:                                                   %
%  Redistribution and use in source and binary forms, with or without modification, are permitted         %
%  provided that the following conditions are met:                                                        %
%     1. Redistributions of source code must retain the above copyright notice, this list of              %
%        conditions and the following disclaimer.                                                         %
%     2. Redistributions in binary form must reproduce the above copyright notice, this list of           %
%        conditions and the following disclaimer in the documentation and/or other materials              %
%        provided with the distribution.                                                                  %
%     3. The name of the author may not be used to endorse or promote products derived from               %
%        this software without specific prior written permission.                                         %
%  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,    %
%  BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE     %
%  ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,  %
%  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS    %
%  OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY        %
%  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR           %
%  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY      %
%  OF SUCH DAMAGE.                                                                                        %
%                                                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% Author: bgermain
%% Created: 16-jan.-2013
%% Description: TODO: Add description to tree_utils
-module(tree_utils).

-export([test/0,next/2,prev/2]).

-spec next(Tree,Key)->Next when
							 Tree::gb_tree(),
							 Key :: term(),
							 Next :: 'nil'|{term(),term()}.

next({S, T},Key) when S=/=0 -> next(nil,T,Key);
next({S, _},_) when S==0 -> nil. 

next(nil, {TKey, _, _, nil},Key) when Key == TKey -> nil;
next(Sol, {TKey, _, _, nil},Key) when Key == TKey-> Sol;
next(_, {TKey, _, _, Bigger},Key) when Key == TKey-> gb_trees:smallest({undefined,Bigger});

next(_, {TKey, TVal, nil, _},Key) when TKey > Key->	{TKey,TVal};
next(_, {TKey, TVal, Smaller, _},Key) when TKey > Key-> next({TKey,TVal},Smaller,Key);

next(Sol,{TKey, _, _, nil},Key) when TKey < Key-> Sol;
next(Sol, {TKey, _, _, Bigger},Key) when TKey < Key-> next(Sol,Bigger,Key).


-spec prev(Tree,Key)->Prev when
							 Tree::gb_tree(),
							 Key :: term(),
							 Prev :: 'nil'|{term(),term()}.
prev({S, T},Key) when S=/=0 -> prev(nil,T,Key);
prev({S, _},_) when S==0 -> nil.

prev(nil,{TKey, _, nil, _},Key) when Key == TKey ->nil;
prev(Sol,{TKey, _, nil, _},Key) when Key == TKey -> Sol;
prev(_,{TKey, _, Smaller, _},Key) when Key == TKey -> gb_trees:largest({undefined,Smaller});

prev(Sol,{TKey, _, nil, _},Key) when TKey > Key-> Sol;
prev(Sol,{TKey, _, Smaller, _},Key) when TKey > Key-> prev(Sol,Smaller,Key);

prev(_,{TKey, TVal, _, nil},Key) when TKey < Key-> {TKey, TVal};
prev(_,{TKey, TVal, _, Bigger},Key) when TKey < Key-> prev({TKey,TVal}, Bigger, Key). 
	
	
	
test()->
	T1 = gb_trees:empty(),
	T2 = gb_trees:enter(7, void, T1),
	T3 = gb_trees:enter(3, void, T2),
	T4 = gb_trees:enter(1, void, T3),
	T5 = gb_trees:enter(5, void, T4),
	T6 = gb_trees:enter(9, void, T5),
	T7 = gb_trees:enter(8, void, T6),
	T8 = gb_trees:enter(10, void, T7),

	%%next(T8,5).
	%%[next(T8, 0),next(T8, 1),next(T8, 2),next(T8, 3),next(T8, 4),next(T8, 5),next(T8, 6),next(T8, 7),next(T8, 8),next(T8, 9),next(T8, 10),next(T8, 11)].
	[prev(T8, 0),prev(T8, 1),prev(T8, 2),prev(T8, 3),prev(T8, 4),prev(T8, 5),prev(T8, 6),prev(T8, 7),prev(T8, 8),prev(T8, 9),prev(T8, 10),prev(T8, 11)].