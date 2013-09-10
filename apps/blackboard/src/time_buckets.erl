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
%% Created: 9-jan.-2013
%% Description: TODO: Add description to cumulative_flow
-module(time_buckets).

-export([add_item/4,remove_item/3,get_cumulative/1]).
-define(MAX_SIZE,70).


get_bucket(Buckets,Cond)->
	{RHead,Bucket,Tail} = get_bucket([], Buckets, Cond),
	{lists:reverse(RHead),Bucket,Tail}.
get_bucket(Head,[],_)->{Head,gb_trees:empty(),[]};
get_bucket(Head,[Bucket|Tail],Cond)->
	case Cond(Bucket) of
		true->{Head,Bucket,Tail};
		false->	get_bucket([Bucket|Head],Tail, Cond)
	end.

add_item(void,{{Time,Key},Val})->{gb_trees:enter({Time,Key}, Val, gb_trees:empty()),void};
add_item(Bucket,{{Time,Key},Val})->
	New_bucket = gb_trees:enter({Time,Key}, Val, Bucket),
	case (gb_trees:size(New_bucket)>?MAX_SIZE) of
		true -> {L_key,L_val,New_bucket2} = gb_trees:take_largest(New_bucket),
				{New_bucket2,{L_key,L_val}};
		false-> {New_bucket,void}
	end.
shift_item_forwards(RHead,void,Tail)->
	lists:reverse(Tail++RHead);
shift_item_forwards(RHead,Item,[])->
	{New_Bucket,void} = add_item(void, Item),
	lists:reverse([New_Bucket|RHead]);
shift_item_forwards(RHead,Item,[Bucket|Tail])->
	{New_Bucket,Rest} = add_item(Bucket, Item),
	shift_item_forwards([New_Bucket|RHead],Rest,Tail).

shift_item_backwards({_,Size},_) when Size==0->
	[];
shift_item_backwards({Bucket,Size},[])when Size=/=0->
	[Bucket];
shift_item_backwards({Bucket,Size},[Next_Bucket|[]]) when Size=/=0->
	{Key,Val,Next_bucket_update} = gb_trees:take_smallest(Next_Bucket),
	Bucket_update = gb_trees:enter(Key, Val, Bucket),
	case gb_trees:size(Next_bucket_update) of
		0 -> [Bucket];
		_ -> [Bucket,Bucket_update]
	end;
shift_item_backwards({Bucket,Size},[Next_Bucket|Tail]) when Size=/=0->
	{Key,Val,Next_bucket_update} = gb_trees:take_smallest(Next_Bucket),
	Bucket_update = gb_trees:enter(Key, Val, Bucket),
	[Bucket_update|shift_item_backwards({Next_bucket_update,gb_trees:size(Next_bucket_update)},Tail)].
  
add_item(Buckets,Time,Key,Value)->
	{Head,Bucket,Tail} = get_bucket(Buckets,fun(Bucket) -> ((gb_trees:size(Bucket)=<?MAX_SIZE) or ({Time,Key}=<element(1,gb_trees:largest(Bucket)))) end),
	shift_item_forwards(lists:reverse(Head), {{Time,Key},Value}, [Bucket|Tail]).
	
remove_item(Buckets,Time,Key)->
	{Head,Bucket,Tail} = get_bucket(Buckets,fun(Bucket) ->{Time,Key}=<element(1,gb_trees:largest(Bucket)) end),
	New_Bucket = gb_trees:delete({Time,Key}, Bucket),
	Head++shift_item_backwards({New_Bucket,gb_trees:size(New_Bucket)}, Tail).

get_cumulative(Buckets)->
	lists:reverse(get_cumulative(Buckets,[],1)).

get_cumulative([Bucket],RCum,N)->[{N * ?MAX_SIZE,element(1,element(1,gb_trees:largest(Bucket)))}|RCum];
get_cumulative([Bucket|Tail],RCum,N)->get_cumulative(Tail,[{N * ?MAX_SIZE,element(1,element(1,gb_trees:largest(Bucket)))}|RCum],N+1).
	
