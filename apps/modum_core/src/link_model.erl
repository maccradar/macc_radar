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
%% Description: TODO: Add description to link_model
-module(link_model).

-export([start_time/2,end_time/2,propagate_flow/3,accommodate_max_capacity/2,cfs_to_png_raw/3, cfs_to_points_raw/1,get_travel_times/1]).

-include("states.hrl").
-include_lib("util/include/debug.hrl").


propagate_flow(down,CF,OldState)->
	{T,_} = cumulative_func:first(x,CF),
	propagate_flow(down,CF, ?undefined, T, OldState);
propagate_flow(up,CF,OldState=#linkBeing{models = #models{fd = FD},blackboard = #blackboard{cf_b=OldCF_B}})->
	{T,_} = cumulative_func:last(x,CF),
	ContraintUp1 = propagate_flow(up,?undefined,CF, T,OldState ),
	{Yf,_} = cumulative_func:first(y, CF),
	{_,S} = cumulative_func:first(y, ContraintUp1),
	Xf = line_segment:y_x(Yf, line_segment:get(x1, S), line_segment:get(y1, S), fundamental_diagram:c(FD)),
	ContraintUp2 = cumulative_func:add_point(Xf, Yf, ContraintUp1).



propagate_flow(up,CF_B,CF_E, T,Oldstate = #linkBeing{state = #linkState{length = Length},models = #models{fd = FD} } ) ->
	Back_Prop_Time = Length/fundamental_diagram:w(FD),
	CarDiff = Length * fundamental_diagram:kjam(FD),
	N = cumulative_func:x_y(T, CF_E),
	New_CF_B = cumulative_func:add_point(T + Back_Prop_Time,N + CarDiff,CF_B),
	case cumulative_func:prevPoint(x, T, CF_E) of
		{PrevT,nil} -> PrevN = cumulative_func:x_y(PrevT, CF_E),
					   cumulative_func:add_point(PrevT + Back_Prop_Time, PrevN + CarDiff,New_CF_B);
		{PrevT,_} -> propagate_flow(up,New_CF_B,CF_E, PrevT , Oldstate)
		
	end;

propagate_flow(down,CF_B,CF_E, T,Oldstate ) ->
	New_CF_E = cumulative_func:add_point(end_time(T, Oldstate),cumulative_func:x_y(T, CF_B),CF_E),
	case cumulative_func:nextPoint(x, T, CF_B) of
		{NextT,nil} -> NextN = cumulative_func:x_y(NextT, CF_B),
					   cumulative_func:add_point(end_time(NextT, Oldstate),NextN,New_CF_E);
		{NextT,_} -> propagate_flow(down,CF_B,New_CF_E, NextT , Oldstate)
	end.

end_time(T,#linkBeing{models=#models{fd=FD},state=#linkState{length=L},blackboard=#blackboard{cf_b=Old_CF_B,cf_e=Old_CF_E}})->
	case min(cumulative_func:size(Old_CF_E),cumulative_func:size(Old_CF_B)) of
		Size when Size > 0 -> 
			case cumulative_flow:end_time( T, Old_CF_B, Old_CF_E) of
				?undefined -> T+L/fundamental_diagram:vf(FD);
				EndTime when EndTime > T -> EndTime;
				_EndTime -> T+L/fundamental_diagram:vf(FD)
			end;
		_Size -> T+L/fundamental_diagram:vf(FD)
	end.

start_time(T,#linkBeing{models=#models{fd=FD},state=#linkState{length=L},blackboard=#blackboard{cf_b=Old_CF_B,cf_e=Old_CF_E}})->
	case min(cumulative_func:size(Old_CF_E),cumulative_func:size(Old_CF_B)) of
		Size when Size > 0 -> 
			case cumulative_flow:start_time( T, Old_CF_B, Old_CF_E) of
				?undefined -> T-L/fundamental_diagram:vf(FD);
				StartTime when StartTime < T -> StartTime;
				_StartTime -> T-L/fundamental_diagram:vf(FD)
			end;
		_Size -> T-L/fundamental_diagram:vf(FD)
	end.
get_travel_times(LB=#linkBeing{blackboard=#blackboard{cf_b=CF_B,cf_e=CF_E}})->
	get_travel_times(LB,[],element(1,cumulative_func:first(x, CF_B)),element(1,cumulative_func:last(x, CF_E))).
		
get_travel_times(LB,TT,T,MAXT) when T>MAXT->
	lists:reverse(TT);
get_travel_times(LB,TT,T,MAXT)->
	get_travel_times(LB,[{T,end_time(T, LB)}|TT],T+1,MAXT).
	

accommodate_max_capacity(CF,G)->
	FS = cumulative_func:first_segment(CF),
	NewFunc= cumulative_func:new(line_segment:get(x1, FS), line_segment:get(y1, FS)),
	accommodate_max_capacity(line_segment:get(x1, FS), CF, element(1, cumulative_func:last(x, CF)), NewFunc, G).
accommodate_max_capacity(X,_,LastX_CF,NewCF,_) when LastX_CF =< X->
	NewCF;
accommodate_max_capacity(X,CF,LastX_CF,NewCF,G)->
	SegAtX = cumulative_func:segment_at(X, CF),
	case line_segment:steeper(SegAtX, G) of
		true -> case cumulative_func:find_next_gradient_intersection(X, CF, {X,cumulative_func:x_y(X, CF),G}) of
					?undefined -> Ylast_orig = line_segment:get(y2, cumulative_func:last_segment(CF) ),
								  case {line_segment:get(x2, cumulative_func:last_segment(NewCF) ),line_segment:get(y2, cumulative_func:last_segment(NewCF) )} of
										{_,Ylast_min} when Ylast_min == Ylast_orig -> NewCF;
										{Xlast_min,Ylast_min} when Ylast_min =< Ylast_orig -> XL = (Xlast_min + ((Ylast_orig - Ylast_min) / G)),
																							  cumulative_func:add_point(XL, Ylast_orig, NewCF);
										{_,Ylast_min} when Ylast_min > Ylast_orig ->  illegal_state
									end;
					Segment -> NewX = line_segment:get(x2, Segment),
							   NewCF_Up = cumulative_func:add_segements([Segment], NewCF),
							   accommodate_max_capacity(NewX, CF, LastX_CF, NewCF_Up,  G)
				end;
		false -> NewCF_Up = cumulative_func:add_segements([SegAtX], NewCF),
				 NewX = line_segment:get(x2, SegAtX),
				 accommodate_max_capacity(NewX, CF, LastX_CF, NewCF_Up,  G);
		?undefined -> NewCF
	end.				


cfs_to_png_raw(Link, Width, Height)->
	{?reply, being, LB} = gen_server:call(Link, being, ?callTimeout),
	CF_B = LB#linkBeing.blackboard#blackboard.cf_b,
	CF_E = LB#linkBeing.blackboard#blackboard.cf_e,
	case {CF_B, CF_E} of
		{?undefined, ?undefined} -> cumulative_func:to_png([{cf_b_undefined,void},{cf_e_undefined,void}], Width, Height);
		{?undefined, _} -> cumulative_func:to_png([{cf_b_undefined,void},{cf_e,CF_E}], Width, Height);
		{_, ?undefined} -> cumulative_func:to_png([{cf_b,CF_B},{cf_e_undefined,void}], Width, Height);
		{_, _} -> cumulative_func:to_png([{cf_b,CF_B},{cf_e,CF_E}], Width, Height)
	end.

cfs_to_points_raw(Link) ->
	{?reply, being, LB} = gen_server:call(Link, being, ?callTimeout),
	CF_B = LB#linkBeing.blackboard#blackboard.cf_b,
	CF_E = LB#linkBeing.blackboard#blackboard.cf_e,
	case {CF_B, CF_E} of
		{?undefined, ?undefined} -> [{cf_b_undefined,[{0,0}]},{cf_e_undefined,[{0,0}]}];
		{?undefined, _} -> cumulative_func:cfs_to_points([{cf_b_undefined,void},{cf_e,CF_E}], []);
		{_, ?undefined} -> cumulative_func:cfs_to_points([{cf_b,CF_B},{cf_e_undefined,void}], []);
		{_, _} -> cumulative_func:cfs_to_points([{cf_b,CF_B},{cf_e,CF_E}], [])
	end.	