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
%% Description: TODO: Add description to cumulative_flow
-module(cumulative_flow).
-include_lib("util/include/consts.hrl").
-export([get_flow/2,end_time/3,test/0,get_cumulative/1,nbr_vehicles/3,test_min/0,constrain_cf/2]).

get_cumulative(Blackboard_points)->
	case Blackboard_points of
		[{N,T}|Points] -> get_cumulative(N,Points,cumulative_func:add_point(T, N, ?undefined));
		_ -> ?undefined
	end.		
	

get_flow(CF,Time)->
	Seg = cumulative_func:segment_at(Time,CF),
	line_segment:gradient(Seg).
	
	
get_cumulative(_,[],CF) ->CF;
get_cumulative(N1,[{N2,_}|Tail],CF) when N1 == N2 -> get_cumulative(N2,Tail,CF);
get_cumulative(_,[{N,T}|Tail],CF) -> get_cumulative(N,Tail,cumulative_func:add_point(T, N, CF)).
	
				 
	
end_time(TB,CF_B,CF_E)->	
	case {element(1,cumulative_func:first(x, CF_B)),element(1,cumulative_func:last(x, CF_B))} of
		{MIN_b,MAX_b} when ((MIN_b =< TB) and (TB =< MAX_b)) -> N = cumulative_func:x_y(TB, CF_B),
																cumulative_func:y_x(N, CF_E);
		_ -> ?undefined
	end.

start_time(TE,CF_B,CF_E)->
    case {element(1,cumulative_func:first(x, CF_E)),element(1,cumulative_func:last(x, CF_E))} of
		{MIN_b,MAX_b} when ((MIN_b =< TE) and (TE =< MAX_b)) -> N = cumulative_func:x_y(TE, CF_E),
																cumulative_func:y_x(N, CF_B);
		_ -> ?undefined
	end.

	
nbr_vehicles(_TimeBegin, CF_B, CF_E) when (CF_B == ?undefined) or (CF_E == ?undefined) ->
	0;
nbr_vehicles(TimeBegin,CF_B,CF_E)->
	cumulative_func:x_y(TimeBegin, CF_B)-cumulative_func:x_y(TimeBegin, CF_E).

filter_cumulative(CF,Delta)->
	First_Seg =  cumulative_func:first_segment(CF),
	case cumulative_func:size(CF) of
		1 -> filter_cumulative(First_Seg,nil, CF, CF, Delta);
		_ -> filter_cumulative(First_Seg,cumulative_func:next_segment(x, line_segment:get(x1, First_Seg), CF), CF, CF, Delta)
	end.
filter_cumulative(_,nil,_,New_CF,_)->
	New_CF;
filter_cumulative(nil,Seg,CF,New_CF,Delta)->
	filter_cumulative(Seg, cumulative_func:segment_at( line_segment:get(x2, Seg), CF), CF, New_CF, Delta);
filter_cumulative(Seg1,Seg2,CF,New_CF,Delta)->
	case {line_segment:gradient(Seg1),line_segment:gradient(Seg2)} of
		{G1,G2} when (abs(G1-G2) >Delta)-> filter_cumulative(Seg2, cumulative_func:segment_at( line_segment:get(x2, Seg2), CF), CF, New_CF, Delta);
		_ ->  New_CF_up = cumulative_func:remove_point(line_segment:get(x1, Seg2),New_CF),
			  NewSeg = cumulative_func:segment_at(line_segment:get(x1, Seg2),New_CF_up),
			  filter_cumulative(NewSeg, cumulative_func:segment_at( line_segment:get(x2, NewSeg), CF), CF, New_CF_up, Delta)
	end.


cfmin(CF1,CF2)->
	cfmin(CF1, CF2, ?undefined, max(element(1,cumulative_func:first(x, CF1)),element(1,cumulative_func:first(x, CF2))), min(element(1,cumulative_func:last(x, CF1)),element(1,cumulative_func:last(x, CF2)))).

cfmin(CF1,CF2,MinCF,X,MaxX) when X == MaxX->
	Y_CF1 = cumulative_func:x_y(X, CF1),
	Y_CF2 = cumulative_func:x_y(X, CF2),
	NewMinCF = cumulative_func:add_point(X, min(Y_CF1,Y_CF2), MinCF),
	NewMinCF;
cfmin(CF1,CF2,MinCF,X,MaxX)->
	SegCF1 = cumulative_func:segment_at(X, CF1),
	SegCF2 = cumulative_func:segment_at(X, CF2),
	Y_CF1 = cumulative_func:x_y(X, CF1),
	Y_CF2 = cumulative_func:x_y(X, CF2),
	NewMinCF = cumulative_func:add_point(X, min(Y_CF1,Y_CF2), MinCF),
	MinNext = min(element(1,cumulative_func:nextPoint(x, X, CF1)),element(1,cumulative_func:nextPoint(x, X, CF2))),
	
	case line_segment:intersection(SegCF1, SegCF2) of
		{?undefined,_} -> cfmin(CF1, CF2,NewMinCF,MinNext,MaxX);
		{_,?undefined} -> cfmin(CF1, CF2,NewMinCF,MinNext,MaxX);
		{X_intersec,_} when (X_intersec > X ) and (X_intersec < MinNext)-> cfmin(CF1, CF2,NewMinCF,X_intersec,MaxX);
		_ -> cfmin(CF1, CF2,NewMinCF,MinNext,MaxX)
	end.
	


	

constrain_cf(CF,?undefined)->
	CF;
constrain_cf(?undefined,_Constraint)->
	?undefined;
constrain_cf(CF,Constraint)->
	CF_min0 = cfmin(CF,Constraint),
	Ylast_orig = line_segment:get(y2, cumulative_func:last_segment(CF) ),
	case line_segment:get(y2, cumulative_func:last_segment(CF_min0) ) of
		Ylast_min when Ylast_min == Ylast_orig -> CF_min0;
		Ylast_min when Ylast_min =< Ylast_orig -> cumulative_func:add_point(cumulative_func:y_x(Ylast_orig, Constraint), Ylast_orig, CF_min0);
		Ylast_min when Ylast_min > Ylast_orig -> io:format("FS_CF = ~w FS_Constraint = ~w ~n",[cumulative_func:first_segment(CF),cumulative_func:first_segment(Constraint)]),
												 cumulative_func:to_file(png,[{cf,CF},{constraint,Constraint},{cf_min,CF_min0}], "./error.png"),
												 illegal_state
	end.

test_min()->
		
	C_F11 = cumulative_func:new(0,0),
	C_F12 = cumulative_func:add_point(5,3, C_F11),
 	C_F13 = cumulative_func:add_point(11, 6, C_F12),
   	C_F14 = cumulative_func:add_point(17, 7, C_F13),
	
	C_F21 = cumulative_func:new(0,0),
	C_F22 = cumulative_func:add_point(5,4, C_F21),
 	C_F23 = cumulative_func:add_point(11, 5, C_F22),
   	C_F24 = cumulative_func:add_point(17, 10, C_F23),
	
%% 	CF_Min = constrain_cf( C_F24,C_F14),
	CF_Min = cfmin( C_F24,C_F14),	
	%CF_F = filter_cumulative(CF_Min, 0.01),
	cumulative_func:to_file(png,[{cf1,C_F14},{cf2,C_F24}], "./cf_test0.png"),
	cumulative_func:to_file(png,[{min,CF_Min}], "./cf_test1.png").
test()->
	io:format("Test started~n"),
	BB = bb_test:pheromone(bb_trafficflow),
	CF = get_cumulative(BB),
	CF_F = filter_cumulative(CF, 5),
	
	cumulative_func:to_file(png,[{test,CF},{filter,CF_F}], "./blackboard_test.png").