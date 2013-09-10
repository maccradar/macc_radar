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
%% Created: 11-jan.-2013
%% Description: TODO: Add description to bucket
-module(list_bucket).

-export([empty/0,enter/3,size/1,largest/1,smallest/1,take_largest/1,take_smallest/1,delete/2]).

-include("blackboard_records.hrl").

empty()->#bucket{}.
enter(Key,Val,Bucket) when Bucket#bucket.size == 0->
	Bucket#bucket{size=1,list=[{Key,Val}],smallest={Key,Val},largest={Key,Val}};
enter(Key,Val,Bucket) when Key >=element(1, Bucket#bucket.largest)->
	Bucket#bucket{size=Bucket#bucket.size+1,list=lists:reverse([{Key,Val}|lists:reverse(Bucket#bucket.list)]),smallest=Bucket#bucket.smallest,largest={Key,Val}};
enter(Key,Val,Bucket) when Key =<element(1, Bucket#bucket.smallest)->
	Bucket#bucket{size=Bucket#bucket.size+1,list=[{Key,Val}|Bucket#bucket.list],smallest={Key,Val},largest=Bucket#bucket.largest};
enter(Key,Val,Bucket)->
	Bucket#bucket{size=Bucket#bucket.size+1,list=add_ordered(Bucket,{Key,Val}),smallest=Bucket#bucket.smallest,largest=Bucket#bucket.largest}.
size(Bucket)->
	Bucket#bucket.size.
largest(Bucket)->
	Bucket#bucket.largest.
smallest(Bucket)->
	Bucket#bucket.smallest.
take_largest(Bucket) when Bucket#bucket.size==1->
	[{Key,Val}] = Bucket#bucket.list,
	{Key,Val,empty()};
take_largest(Bucket) when Bucket#bucket.size==2->
	[Smallest,Largest] = Bucket#bucket.list,
	{element(1, Largest),element(2, Largest),Bucket#bucket{size=1,list=[Smallest],largest= Smallest}};
take_largest(Bucket)->
	[Largest|[NewLargest|Tail]]= lists:reverse(Bucket#bucket.list),
	{element(1, Largest),element(2, Largest),Bucket#bucket{size=Bucket#bucket.size-1,list=lists:reverse([NewLargest|Tail]),largest=NewLargest}}.
take_smallest(Bucket) when Bucket#bucket.size==1->
	[{Key,Val}] = Bucket#bucket.list,
	{Key,Val,empty()};
take_smallest(Bucket) when Bucket#bucket.size==2->
	[Smallest,Largest] = Bucket#bucket.list,
	{element(1, Smallest),element(2, Smallest),Bucket#bucket{size=1,list=[Largest],smallest=Largest}};
take_smallest(Bucket)->
	[Smallest|[Second|Tail]]= Bucket#bucket.list,
	{element(1, Smallest),element(2, Smallest),Bucket#bucket{size=Bucket#bucket.size-1,list=[Second|Tail],smallest= Second}}.

delete(Key,Bucket) when Key == element(1,Bucket#bucket.smallest)->
	element(3,take_smallest(Bucket));
delete(Key,Bucket) when Key == element(1,Bucket#bucket.largest)->
	element(3,take_largest(Bucket));
delete(Key,Bucket)->
	Bucket#bucket{size = Bucket#bucket.size-1,list = delete(1,Key,[],Bucket#bucket.list,0,Bucket#bucket.size)}. 


delete(TPlace,Key,RHead, [Current],_,_) when Key == element(TPlace, Current)->
	lists:reverse(RHead);
delete(TPlace,Key,RHead, [Current],_,_) when Key =/= element(TPlace, Current)->
	lists:reverse([RHead|[Current]]); 
delete(TPlace,Key,RHead, [Current|Tail],HeadSize,TotSize) when Key =/= element(TPlace, Current)->
	delete(TPlace, Key, [Current|RHead], Tail, HeadSize+1, TotSize);
delete(TPlace,Key,RHead, [Current|Tail],HeadSize,TotSize) when ((Key == element(TPlace, Current)) and (HeadSize < (TotSize - HeadSize)))->
	lists:reverse(RHead)++Tail;
delete(TPlace,Key,RHead, [Current|Tail],HeadSize,TotSize) when ((Key == element(TPlace, Current)) and (HeadSize >= (TotSize - HeadSize)))->
	lists:reverse(lists:reverse(Tail)++RHead).

add_ordered(Bucket,Item)->
	add_ordered(1,[],Bucket#bucket.list,0,Bucket#bucket.size,Item).
add_ordered(_,RHead,[],_,_,Item)->
	lists:reverse([Item|RHead]);
add_ordered(TPlace,RHead,[Current|Tail],HeadSize,TotSize,Item) when element(TPlace, Item) > element(TPlace, Current)->
	add_ordered(TPlace,[Current|RHead],Tail,HeadSize+1,TotSize,Item);
add_ordered(TPlace,RHead,[Current|Tail],HeadSize,TotSize,Item) when ((element(TPlace, Item) =< element(TPlace, Current)) and (HeadSize >= (TotSize - HeadSize)))->
	lists:reverse(lists:reverse([Item|[Current|Tail]])++RHead);
add_ordered(TPlace,RHead,[Current|Tail],HeadSize,TotSize,Item) when ((element(TPlace, Item) =< element(TPlace, Current)) and (HeadSize < (TotSize - HeadSize)))->
	lists:reverse(RHead)++[Item|[Current|Tail]];
add_ordered(TPlace,RHead,List,HeadSize,TotSize,Item) ->
	io:format("error"),
	io:format("add ordered error: ~w ~w ~w ~w ~w ~w" ,[TPlace,RHead,List,HeadSize,TotSize,Item]).
