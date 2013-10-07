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
%% Created: 18-jan.-2013
%% Description: represents a cumulative function with a discrete ascending monotone function  
%%time always starts at 0

-module(cumulative_func).

-include("consts.hrl").
-export([update_last_point/4,multiply/3, multiply/4, add_point/3,add_points/2,y_x/2,x_y/2,update_point/4,new/2,test/0,next/3,prev/3,first/2,next_segment/3,next_value/3,first_segment/1,prev_segment/3,prev_value/3,cf_to_points/1,to_file/3,segment_at/2,last/2,last_segment/1,size/1,remove_point/2,nextPoint/3,prevPoint/3,shift_from/4,add_segments/2,find_next_gradient_intersection/3,delete_after/3,to_png/3, cfs_to_points/2]).
-export_type([func/0]).
%% representation of a cumulative function (strict ascending monotone function)
%%     - binary tree mapping x values to segments
%%     - binary tree mapping y values to segments
-record(func,{x_y=gb_trees:empty() :: gb_tree(),
              y_x=gb_trees:empty() :: gb_tree()}).
-type func() :: #func{}.

size(#func{x_y = X_Y})->
	gb_trees:size(X_Y);
size(_)->
	0.
segment_at(X,CF = #func{x_y = X_Y})->
	case gb_trees:lookup(X, X_Y) of
		none -> PrevSeg = prev_segment(x, X, CF),
			    case line_segment:get(x2,PrevSeg) of
					X2 when X2== X -> nil;
					_ -> PrevSeg
				end;
		{value,S} ->S
	end.

add_segments([],CF)->
	CF;
add_segments([S|T],CF)->
	NewCF = cumulative_func:add_point(line_segment:get(x2, S), line_segment:get(y2, S), CF),
	add_segments(T, NewCF).

find_next_gradient_intersection(X,CF,{X0,Y0,G}) when is_number(X0) and is_number(Y0) and is_number(G)->
	S = cumulative_func:segment_at(X, CF),
	find_next_gradient_intersection(X,S,CF,{X0,Y0,G},nil).

find_next_gradient_intersection(XStart, nil,_,{X0,_,_},ACC_segment) when X0 == XStart ->
	ACC_segment;
find_next_gradient_intersection(_, nil,_,_,_)->
	?undefined;
find_next_gradient_intersection(XStart,S ,CF,{X0,Y0,G},ACC_segment)  when X0 == XStart->	
	X2seg = line_segment:get(x2, S),
	find_next_gradient_intersection(X2seg,cumulative_func:segment_at(X2seg, CF),CF,{X0,Y0,G},line_segment:join(ACC_segment, S));
find_next_gradient_intersection(XStart,S ,CF,{X0,Y0,G},ACC_segment)->	
	X2seg = line_segment:get(x2, S),
	case line_segment:intersection(S, X0, Y0, G) of
		{_,?undefined} -> find_next_gradient_intersection(X2seg,cumulative_func:segment_at(X2seg, CF),CF,{X0,Y0,G},line_segment:join(ACC_segment, S));
		{X_I,Y_I} when X_I > XStart-> New_S = line_segment:new(line_segment:get(x1, S), line_segment:get(y1, S),X_I, Y_I),
									  line_segment:join(ACC_segment, New_S);
		{_,_} -> find_next_gradient_intersection(X2seg,cumulative_func:segment_at(X2seg, CF),CF,{X0,Y0,G},line_segment:join(ACC_segment, S))
	end.

% next point with corresponding segment 
nextPoint(Type, T, CF)->
	case cumulative_func:next(Type, T, CF) of
		nil -> {element(1,cumulative_func:last(Type,CF)),nil};
		NextPoint -> NextPoint
	end.

prevPoint(Type, T, CF)->
	case cumulative_func:prev(Type, T, CF) of
		nil -> {element(1,cumulative_func:first(Type,CF)),nil};
		NextPoint -> NextPoint
	end.

nextXYPoint({X,_Y}, CF)->
	case {cumulative_func:next(x, X, CF),element(1,cumulative_func:last(x,CF))} of
		{nil,Last} when Last =< X -> {?undefined,?undefined};
		{nil,{_,Seg}} -> {line_segment:get(x1, Seg),line_segment:get(y1, Seg)};
		{{_,Seg},_} -> {line_segment:get(x1, Seg),line_segment:get(y1, Seg)}
	end.

%% updates an existing point in the cumulative.
update_last_point(y,LastX,Y_upd,Func=#func{x_y=X_Y,y_x=Y_X})->
	{X,S_a} = prev(x,LastX,Func),
	S_a_upd = line_segment:update(y2, Y_upd, S_a),
	X_Y_u1 = gb_trees:update(X, S_a_upd, X_Y),
	Y = line_segment:x_y(X, S_a),
	io:format("Y_X: ~w ~w~n", [Y, Y_X]),
	Y_X_u1 = gb_trees:delete(Y,Y_X),
	Y_X_u2 = gb_trees:enter(Y_upd, S_a_upd, Y_X_u1),
	#func{x_y = X_Y_u1,y_x = Y_X_u2}.

shift_point(y,X,Y,Val,CF)->
	update_point(y, X, Y+Val, CF).
	
shift_from(_,{?undefined,?undefined},_,CF)->
	CF;
shift_from(Type,{CurX,CurY},Value,CF)->
	shift_from(y,nextXYPoint({CurX,CurY},CF), Value, shift_point(Type, CurX, CurY,Value,CF)).
	

%% returns the largest X value with corresponding segment
%% assumes a non empty cumul func
-spec last(Type,Func) -> {Key, Val} when
      Type :: 'x'|'y',
      Func :: #func{},
      Key :: term(),
	  Val :: term().
last(x,#func{x_y = X_Y})->
	{_,L_seg} = gb_trees:largest(X_Y),
	{line_segment:get(x2, L_seg),L_seg};
%% returns the largest Y value with corresponding segment 
%% assumes a non empty cumul func
last(y,#func{y_x = Y_X})->
	{_,L_seg} = gb_trees:largest(Y_X),
	{line_segment:get(y2, L_seg),L_seg}.
%% returns the segment corresponding to the last x_y point
%% assumes a non empty cumul func
last_segment(#func{x_y = X_Y})->
	{_ ,S} = gb_trees:largest(X_Y),
	S.

%% returns the smallest X value with corresponding segment {smallest_x,first_segment}
%% assumes a non empty cumul func
-spec first(Type,Func) -> {Key, Val} when
      Type :: 'x'|'y',
      Func :: #func{},
      Key :: term(),
	  Val :: term().
first(x,#func{x_y = X_Y})->
	gb_trees:smallest(X_Y);
%% returns the smallest Y value with corresponding segment {smallest_y,first_segment}
%% assumes a non empty cumul func
first(y,#func{y_x = Y_X})->
	gb_trees:smallest(Y_X).
%% returns the segment corresponding to the first x_y point
%% assumes a non empty cumul func
first_segment(#func{x_y = X_Y})->
	{_ ,S} = gb_trees:smallest(X_Y),
	S.

%% returns the next discrete value/segment of the cummulative (either X or Y values depending on given atom as first argument)
%%      - {axis_value, corresponding_segement}
next(x, X,#func{x_y = X_Y})->
	tree_utils:next(X_Y, X);
next(y,Y,#func{y_x=Y_X})->
	tree_utils:next(Y_X, Y).
prev(x, X,#func{x_y = X_Y})->
	tree_utils:prev(X_Y, X);
prev(y,Y,#func{y_x=Y_X})->
	tree_utils:prev(Y_X, Y).

%% returns the next segement of the cummulative (either X or Y values depending on given atom as first argument)
next_segment(Type, Cur,CF)->
	case next(Type, Cur, CF) of
		nil -> nil;
		{_,S}->S
	end.

%% returns the prev segement of the cummulative (either X or Y values depending on given atom as first argument)
prev_segment(Type, Cur,CF)->
	case prev(Type, Cur, CF) of
		nil -> nil;
		{_,S}->S
	end.

%% returns the next discrete value of the cummulative (either X or Y values depending on given atom as first argument)
%%      -axis_value
next_value(Type,Cur,CF)->
	case next(Type, Cur, CF) of
		nil -> nil;
		{V,_}->V
	end.
	

%% returns the prev discrete value of the cummulative (either X or Y values depending on given atom as first argument)
%%      -axis_value
prev_value(Type,Cur,CF)->
	case prev(Type, Cur, CF) of
		nil -> nil;
		{V,_}->V
	end.

%% creates a new cummulative function
%% X and Y are the coordinates of the starting point 
new(X,Y)->
	S = line_segment:new(X,Y, X, Y),
	XY = gb_trees:enter(X, S, gb_trees:empty()),
	YX = gb_trees:enter(Y, S, gb_trees:empty()),
	#func{x_y = XY,y_x = YX}.

%% returns the y value given an X value. returns the first/last Y value when X is outside the range of the cumulative
x_y(X, F = #func{x_y =X_Y})->
	case gb_trees:lookup(X, X_Y) of
		{value,S} -> min(line_segment:x_y(X, S),line_segment:get(y2, S));
		none -> case tree_utils:prev(X_Y, X) of
					{_,S}->min(line_segment:x_y(X, S),line_segment:get(y2, S)); % minimum of y2 and 'undefined' if X > x2 of segment.
					nil -> line_segment:get(y1,first_segment(F))
				end 
	end.

%% returns the y value given an X value. returns the undefined when Y is outside the range of the cumulative
y_x(Y, #func{y_x =Y_X})->
	case gb_trees:lookup(Y, Y_X) of
		{value,S} -> line_segment:y_x(Y, S);
		none -> case tree_utils:prev(Y_X, Y) of
					{_,S}->line_segment:y_x(Y, S);
					nil->?undefined
				end
	end.

%% updates an existing point in the cumulative.
update_point(y,X,Y_upd,Func=#func{x_y=X_Y,y_x=Y_X})->
	case gb_trees:lookup(X, X_Y) of 
		none -> update_last_point(y,X,Y_upd,Func);
		{value,S_a} ->
	S_a_upd = line_segment:update(y1, Y_upd, S_a),
	X_Y_u1 = gb_trees:update(X, S_a_upd, X_Y),
	Y = line_segment:x_y(X, S_a),
	case tree_utils:prev( X_Y,X) of
		{X_b,S_b} -> S_b_upd = line_segment:update(y2, Y_upd, S_b),
					 X_Y_u2 = gb_trees:update(X_b, S_b_upd, X_Y_u1),
					 Y_b = line_segment:x_y(X_b, S_b),
					 Y_X_u1 = gb_trees:update(Y_b, S_b_upd, Y_X),
					 Y_X_u2 = gb_trees:delete(Y,Y_X_u1),
					 Y_X_u3 = gb_trees:enter(Y_upd, S_a_upd, Y_X_u2),
					 #func{x_y = X_Y_u2,y_x = Y_X_u3};
		nil -> Y_X_u1 = gb_trees:delete(Y,Y_X),
			   Y_X_u2 = gb_trees:enter(Y_upd, S_a_upd, Y_X_u1),
			   #func{x_y = X_Y_u1,y_x = Y_X_u2}
	end
	end.


	
add_points([],CF)->
	CF;
add_points([{X,Y}|Points],CF)->
	add_points(Points, add_point(X,Y, CF)).
add_point(x,X,Func)->
  add_point(X, x_y(X, Func),Func);
add_point(y,Y,Func)->
  add_point(y_x(Y, Func),Y ,Func);
add_point(X,Y,Func=#func{x_y=X_Y,y_x=Y_X})->
	{X1_l,S_l} = gb_trees:largest(X_Y),
	case {gb_trees:smallest(X_Y),line_segment:get(x2, S_l)} of
		{{Xsmall, _},_} when Xsmall == X -> update_point(y, X, Y, Func);
		{_,Xlarg} when Xlarg == X -> Y1_l = line_segment:get(y1, S_l),
									 New_Seg = line_segment:new(X1_l,Y1_l,X,Y),
									 X_Y_u = gb_trees:enter(X1_l, New_Seg, X_Y),
									 Y_X_u = gb_trees:enter(Y1_l,New_Seg , Y_X),
									 #func{x_y=X_Y_u,y_x=Y_X_u};
		{{X_s,_},X_l} when ( (X_s < X) and (X < X_l) ) -> insert_point(X,Y,Func);
		{{X_s,S_s},_} when X_s > X -> New_Seg = line_segment:new(X, Y, X_s, line_segment:x_y(X_s, S_s)),
								  X_Y_u = gb_trees:enter(X, New_Seg, X_Y),
								  Y_X_u = gb_trees:enter(Y, New_Seg, Y_X),
								  #func{x_y=X_Y_u,y_x=Y_X_u};
		{_,X_l} when X > X_l ->Y_l = line_segment:get(y2, S_l),
							   New_Seg = line_segment:new(X_l, Y_l, X, Y),
							   X_Y_u = gb_trees:enter(X_l, New_Seg, X_Y),
							   Y_X_u = gb_trees:enter(Y_l, New_Seg, Y_X),
							   #func{x_y=X_Y_u,y_x=Y_X_u}
	end;
add_point(X, Y, _)->
	new(X, Y).

delete_before(y,Y,Func=#func{y_x=Y_X})->
	X = y_x(Y, Func),
	add_point(x, X, Func),
	delete_before(X, element(1, gb_trees:smallest(Y_X)), Func);
delete_before(x,X,Func=#func{x_y=X_Y})->
	add_point(x, X, Func),
	delete_before(X, element(1, gb_trees:smallest(X_Y)), Func);
delete_before(X,XSmallest, #func{x_y=X_Y,y_x=Y_X}) when X > XSmallest->
	{_,_,X_Y_up} = gb_trees:take_smallest(X_Y),
	{_,_,Y_X_up} = gb_trees:take_smallest(Y_X),
	delete_before(X, element(1, gb_trees:smallest(X_Y_up)),#func{x_y=X_Y_up,y_x=Y_X_up});
delete_before(_X,_XLargest,Func) ->
  Func.

delete_after(y,Y,Func=#func{y_x=Y_X})->
	
	X = y_x(Y, Func),
	add_point(x, X, Func),
	delete_after(X, element(1, gb_trees:largest(Y_X)), Func);
delete_after(x,X,Func=#func{x_y=X_Y})->
	add_point(x, X, Func),
	delete_after(X, element(1, gb_trees:largest(X_Y)), Func);
delete_after(X,XLargest,Func = #func{x_y=X_Y,y_x=Y_X}) when X < XLargest ->
	{_,_,X_Y_up} = gb_trees:take_largest(X_Y),
	{_,_,Y_X_up} = gb_trees:take_largest(Y_X),
	delete_after(X, element(1, gb_trees:largest(X_Y_up)),#func{x_y=X_Y_up,y_x=Y_X_up});
delete_after(_X,_XLargest,Func) ->
  Func.
								   
remove_point(X,Func=#func{x_y=X_Y,y_x=Y_X})->
	S = segment_at(X, Func),
	X_Y_u1 = gb_trees:delete(X, X_Y),
	Y_X_u1 = gb_trees:delete(line_segment:get(y1, S), Y_X),
	case prev_segment(x, X, Func) of 
		nil ->  #func{x_y=X_Y_u1,y_x=Y_X_u1};
		S_prev -> S_new = line_segment:join(S_prev, S),
				  X_Y_u2 = gb_trees:enter(line_segment:get(x1, S_prev), S_new, X_Y_u1),
				  Y_X_u2 = gb_trees:enter(line_segment:get(y1, S_prev), S_new, Y_X_u1),
				  #func{x_y=X_Y_u2,y_x=Y_X_u2}
	end.
				   

insert_point(X,Y,Func=#func{x_y=X_Y,y_x=Y_X})->
	case gb_trees:is_defined(X, X_Y) of
		true-> update_point(y,X,Y,Func);
		false->	{X1,S} = tree_utils:prev(X_Y, X),
				case line_segment:split_x(S, X, Y) of
					{S1,S2} -> X_Y_u1 = gb_trees:update(X1, S1, X_Y),
							   X_Y_u2 = gb_trees:enter(X, S2, X_Y_u1),
							   Y1 = line_segment:x_y(X1, S),
							   Y_X_u1 = gb_trees:update(Y1, S1, Y_X),
							   Y_X_u2 = gb_trees:enter(Y, S2, Y_X_u1),
							   #func{x_y=X_Y_u2,y_x=Y_X_u2};
					_ -> Func
				end
	end.

to_file(text,[{Name,#func{x_y=X_Y}}],File)->
	
	Points = cf_to_points(gb_trees:to_list(X_Y)),
	PrintPoint = fun({X,Y}) -> file:write_file(File,io_lib:fwrite("~p ~p \n\n", [X,Y]),[append]),
							   file:write_file(File,io_lib:nl(),[append])
				 end,
	lists:foreach(PrintPoint, Points);
	
	
to_file(png,CFs,File)->
	case cfs_to_points(CFs,[]) of
		[] -> empty;
		Point_CFs -> PNG = egd_chart:graph(Point_CFs,[{width, 1000}, {height, 1000}]),
					 file:write_file(File,PNG)
	end.
	
to_png(CFs,Width, Height)->
	case cfs_to_points(CFs,[]) of
		[] -> Names = [{Name, [{0,0},{0,0}]} || {Name, _} <- CFs], egd_chart:graph(Names, [{width, Width}, {height, Height}]);
		Point_CFs -> egd_chart:graph(Point_CFs,[{width, Width}, {height, Height}])
	end.
cfs_to_points([],Acc)->
	lists:reverse(Acc);
cfs_to_points([{_,void}|Tail],Acc)->
	cfs_to_points(Tail, Acc);
cfs_to_points([{_,?undefined}|Tail],Acc)->
	cfs_to_points(Tail, Acc);
cfs_to_points([{Name,#func{x_y=X_Y}}|Tail],Acc)->
	Points = cf_to_points(gb_trees:to_list(X_Y)),
	cfs_to_points(Tail, [{Name,Points}|Acc]).
											 
cf_to_points([{X,S}|Tail])->
	cf_to_points([{X,line_segment:get(y1, S)}],[{X,S}|Tail]).
cf_to_points(CF_Points,[])->
	lists:reverse(CF_Points);
cf_to_points(CF_Points,[{_,S}|Tail])->
	cf_to_points([{line_segment:get(x2, S),line_segment:get(y2, S)}|CF_Points],Tail).
	
	
multiply(y, Factor, Func=#func{}) ->
	{First,_} = first(x, Func),
	multiply(y, First, Factor, Func).
multiply(y, nil, Factor, Func) ->
	{X, _} = last(x, Func),
	Y = x_y(X, Func),
	NewY = round(Y * Factor),
	update_point(y, X, NewY, Func);
multiply(y, X, Factor, Func=#func{}) ->
	io:format("multiply y: ~w, ~w, ~p~n",[X,Factor, cf_to_points(gb_trees:to_list(Func#func.x_y))]),
	Y = x_y(X, Func),
	NewY = round(Y * Factor),
	NewCF = update_point(y, X, NewY, Func),
	io:format("new CF: ~p~n", [cf_to_points(gb_trees:to_list(NewCF#func.x_y))]),
	NextX = next_value(x,X,NewCF),
	multiply(y, NextX , Factor, NewCF).

test()->
	C_F11 = new(0,0),
	C_F12 = add_point(5,3, C_F11),
 	C_F13 = add_point(11, 6, C_F12),
   	C_F14 = add_point(17, 7, C_F13),
	
	C_F21 = new(0,0),
	C_F22 = add_point(5,4, C_F21),
 	C_F23 = add_point(11, 5, C_F22),
   	C_F24 = add_point(17, 10, C_F23),
	
	P1 = cf_to_points(gb_trees:to_list(C_F14#func.x_y)),
	io:format("Before: ~p~n", [P1]),
	io:format("Last: ~w~n", [gb_trees:lookup(17, C_F14#func.x_y)]),
	NewCF = multiply(y, 0.5, C_F14),
	P2 = cf_to_points(gb_trees:to_list(NewCF#func.x_y)),
	io:format("After: ~p~n", [P2]).
	

