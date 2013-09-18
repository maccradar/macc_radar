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
-module(time_buckets_lists).

-export([add_item/4,remove_item/3,get_cumulative/1,replace_item/6]).
-define(MAX_SIZE,10).
%%-define(MAX_SIZE,2).

get_bucket(Buckets,Cond)->
	{RHead,Bucket,Tail} = get_bucket([], Buckets, Cond),
	{lists:reverse(RHead),Bucket,Tail}.
get_bucket(Head,[],_)->{Head,list_bucket:empty(),[]};
get_bucket(Head,[Bucket|Tail],Cond)->
	case Cond(Bucket) of
		true->{Head,Bucket,Tail};
		false->	get_bucket([Bucket|Head],Tail, Cond)
	end.

add_item(void,{{Time,Key},Val})->{list_bucket:enter({Time,Key}, Val, list_bucket:empty()),void};
add_item(Bucket,{{Time,Key},Val})->
	New_bucket = list_bucket:enter({Time,Key}, Val, Bucket),
	case (list_bucket:size(New_bucket)>?MAX_SIZE) of
		true -> {L_key,L_val,New_bucket2} = list_bucket:take_largest(New_bucket),
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

	
shift_item_backwards({_,Size},_,ACC) when Size==0->
	lists:reverse(ACC);
shift_item_backwards({Bucket,Size},[],ACC)when Size=/=0->
	lists:reverse([Bucket|ACC]);
shift_item_backwards({Bucket,Size},[Next_Bucket|[]],ACC) when Size=/=0->
	{Key,Val,Next_bucket_update} = list_bucket:take_smallest(Next_Bucket),
	Bucket_update = list_bucket:enter(Key, Val, Bucket),
	case list_bucket:size(Next_bucket_update) of
		0 -> lists:reverse([Bucket_update|ACC]);
		_ -> lists:reverse([Next_bucket_update|[Bucket_update|ACC]])
	end;
	
shift_item_backwards({Bucket,Size},[Next_Bucket|Tail],ACC) when Size=/=0-> %%TODO opgepast net tail recursion
	{Key,Val,Next_bucket_update} = list_bucket:take_smallest(Next_Bucket),
	Bucket_update = list_bucket:enter(Key, Val, Bucket),
	shift_item_backwards({Next_bucket_update,list_bucket:size(Next_bucket_update)},Tail, [Bucket_update|ACC]).

% replace_item(Buckets,OldTime,NewTime,OldKey,NewKey,Value)->
	% {Head,Bucket,Tail} = get_bucket(Buckets,fun(Bucket) ->{OldTime,OldKey}=<element(1,list_bucket:largest(Bucket)) end),
	% case {list_bucket:smallest(Bucket),list_bucket:largest(Bucket)} of
		% {From,To} when ((NewTime >= From) and (NewTime =< To )) -> 	New_Bucket = list_bucket:delete({OldTime,OldKey}, Bucket),
																	% list_bucket:enter({NewTime,NewKey},Value,New_Bucket);
		% _ ->remove_item(Buckets,OldTime,OldKey),
			% add_item(Buckets,NewTime,NewKey,Value)
	% end.
replace_item(Buckets,OldTime,NewTime,OldKey,NewKey,Value)->
	{Head,Bucket,Tail} = get_bucket(Buckets,fun(Bucket) ->{OldTime,OldKey}=<element(1,list_bucket:largest(Bucket)) end),
	case {list_bucket:smallest(Bucket),list_bucket:largest(Bucket)} of
		{From,To} when ((NewTime >= From) and (NewTime =< To )) ->  
			BucketT1 = list_bucket:delete({OldTime,OldKey}, Bucket),
			BucketT2 = list_bucket:enter({NewTime,NewKey},Value,BucketT1),
			Head ++ [BucketT2|Tail];
		_ ->BucketsTemp = remove_item(Buckets,OldTime,OldKey),
			add_item(BucketsTemp,NewTime,NewKey,Value)
end.
		
		
add_item(Buckets,Time,Key,Value)->
	{Head,Bucket,Tail} = get_bucket(Buckets,fun(Bucket) -> ((list_bucket:size(Bucket)=<?MAX_SIZE) or ({Time,Key}=<element(1,list_bucket:largest(Bucket)))) end),
	shift_item_forwards(lists:reverse(Head), {{Time,Key},Value}, [Bucket|Tail]).
	
remove_item(Buckets,Time,Key)->
	{Head,Bucket,Tail} = get_bucket(Buckets,fun(Bucket) ->{Time,Key}=<element(1,list_bucket:largest(Bucket)) end),
	New_Bucket = list_bucket:delete({Time,Key}, Bucket),
	case list_bucket:size(New_Bucket) of
		0 -> Head ++ Tail;
		_ -> NewTail = shift_item_backwards({New_Bucket,list_bucket:size(New_Bucket)}, Tail,[]),
			 Head ++NewTail
	end.
	
get_cumulative([])->
	[];
get_cumulative([Bucket|Tail])->
	[{0,element(1,element(1,list_bucket:smallest(Bucket)))}|lists:reverse(get_cumulative([Bucket|Tail],[],1))].

get_cumulative([],RCum,_)->RCum;
get_cumulative([Bucket],RCum,N)->[{N * ?MAX_SIZE,element(1,element(1,list_bucket:largest(Bucket)))}|RCum];
get_cumulative([Bucket|Tail],RCum,N)->get_cumulative(Tail,[{N * ?MAX_SIZE,element(1,element(1,list_bucket:largest(Bucket)))}|RCum],N+1).
	
