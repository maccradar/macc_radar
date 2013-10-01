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
-module(node_model).
-export([redistribute_flows/4,test_sub/1,test/0]).

get_fractions(CFs=[{_I,_J,_CF}],T)->
	Flows = [{I,J,cumulative_flow:get_flow(CF,T)}||{I,J,CF}<-CFs],
	TotalFlow = lists:foldl(fun({_,_,F},TF)->F+TF end,0,Flows),
	[{I,J,F/TotalFlow}||{I,J,F}<-Flows].
	
get_supply(CFs=[{_I,_J,_CF}],T)->
	[{I,lists:sum([cumulative_flow:get_flow(CF,T)||{I1,_,CF}<-CFs,I1=I])}||{I,_,_}<-CFs].
	

%% Cap_i : [{i,Cap}]
%% TurnFrac_ij : [{I,J,Frac}]
%% return : [{i,j,Cap_ij}]
c_ij(Cap_i,TurnFrac_ij)->
	[{I1,J,Fij*Ci}||{I1,Ci}<-Cap_i,{I2,J,Fij}<-TurnFrac_ij,I1==I2].
	
%% Js : [j]
%% C_ij : [{i,j,Cap_ij}]
%% Rreduced_j: [{j,supply_cap}]
%% U_j : [{j,[i]}]
%% return : [{a,j}]
a_j(Js,C_ij,Rreduced_j,U_j)->
	Fun_A = fun
		(_,[],_)-> 1;
		(R,U,J) -> R/(lists:sum([C||{I,J0,C}<-C_ij,J0==J,I2 <- U ,I == I2]))
		end,
	
	[{Fun_A(R,element(2,lists:keyfind(J,1,U_j)),J),J}||J<-Js,{J1,R}<-Rreduced_j,J==J1].


%% A : [{a,j}]	
%% J : [j]
%% return : {a,J}
min_a(A,J)-> 
	lists:min([{A0,J0}||{A0,J0}<-A,J1<-J , J1== J0]).
	
%% U_j : [{j,[i]}]
%% S_i : [{i,s}]
%% Jtilde : j
%% Aj : [{a,j}]
%%  Cap_i : [{i,Cap}]
is_demand_constrained(Uj,Si,Jtilde,Aj,Capi)->
	{_,U} = lists:keyfind(Jtilde,1,Uj),
	{A,_} = lists:keyfind(Jtilde,2,Aj),
	[I0||I0<-U,{I1,S}<-Si,I1==I0,{I3,C}<-Capi,I0==I3,C*A >= S].

%% Is : [i]
%% S_i : [{i,s}]
%% Fij : [{I,J,Frac}]
%% Qold : [{i,j,q}]
%% return [{i,j,q}]
update_q(demand,Is,S_i,Fij,Qold)->
	lists:append(Qold,[{I,J,S*F}||I<-Is,{I1,S}<-S_i,I==I1,{I2,J,F}<-Fij,I == I2]).

%% Jr : most restrictive J
%% U_j : [{j,[i]}]
%% C_ij : [{i,j,Cap_ij}]
%% Aj : [{a,j}]
%% Qold : [{i,j,q}]
%% return [{i,j,q}]
 update_q(oriented_cap,Jr,Uj,Cij,Aj,Qold)->
	Ar = element(1,lists:keyfind(Jr,2,Aj)),
	U = element(2,lists:keyfind(Jr,1,Uj)),
	lists:append(Qold,[{I1,J1,Ar*C}||I0<-U,{I1,J1,C}<-Cij,I0==I1]).
	

%% Is : [i]
%% J : [j]
%% Rreduced [{j,supply_cap}]
%% Q : [{i,j,q}]
%% U_j : [{j,[i]}]
%% return : R[{j,supplycap}],U[{j,[i]},J[j]
update_demand_constrained(Is,Js,Rreduced,Qij,U_j)->
	Rup = lists:foldl(fun(I,Rup)->[{J0,Cap-Q}||{J0,Cap}<-Rup,{I1,J1,Q}<-Qij,I==I1,J0==J1,J<-Js,J0==J] end,Rreduced,Is),
	Uup = lists:foldl(fun(I,Uup)->[{J0,lists:delete(I, U)}||{J0,U}<-Uup,J<-Js,J0==J] end,U_j,Is),
	Jupfun =
		fun(J,Jup)->
			case lists:keyfind(J,1,Uup) of
				{_,[]} -> lists:delete(J,Jup);
				_ -> Jup
			end
		end,
	Jup = lists:foldl(Jupfun,Js,Js),
	{Rup,Uup,Jup}.
update_supply_constrained(Js,Rreduced,Qij,Jr,U_j)->
	Ur = element(2,lists:keyfind(Jr,1,U_j)),
	Rup = lists:foldl(fun(I,Rup)->[{J0,Cap-Q}||{J0,Cap}<-Rup,{I1,J1,Q}<-Qij,I==I1,J0==J1,J<-Js,J0==J] end,Rreduced,Ur),
	Uup = [{J0,lists:subtract(U, Ur)}||{J0,U}<-U_j,J<-Js,J0==J],
	Uup2 = lists:keyreplace(Jr,1,Uup,{Jr,Ur}),
	Jupfun =
		fun(J,Jup)->
			case lists:keyfind(J,1,Uup2) of
				{_,[]} -> lists:delete(J,Jup);
				_ -> Jup
			end
		end,
	Jup = lists:foldl(Jupfun,Js,Js),	
	Jup2 = lists:delete(Jr,Jup),
	{Rup,Uup2,Jup2}.
	

redistribute_flows(Si,Fij,Ci,Rj)->
	Cij = c_ij(Ci,Fij),
	Uj = [{J,[I||{I,J1,F}<-Fij,J==J1,F>0]}||{J,_}<-Rj],
	J = [J||{J,_}<-Rj],
	redistribute_flows(Si,Fij,Rj,Ci,Cij,Uj,J,[]).
redistribute_flows(Si,Fij,Rj,Ci,Cij,Uj,[],Qij)->
	Qij;
redistribute_flows(Si,Fij,Rj,Ci,Cij,Uj,J,Qij)->
	A = a_j(J,Cij,Rj,Uj),
	{Amin,Jr}  = min_a(A,J),
	case is_demand_constrained(Uj,Si,Jr,A,Ci) of
		[] -> 
			Qup = update_q(oriented_cap,Jr,Uj,Cij,A,Qij),
			{Rup,Uup,Jup} = update_supply_constrained(J,Rj,Qup,Jr,Uj),
			redistribute_flows(Si,Fij,Rup,Ci,Cij,Uup,Jup,Qup);
		I -> 
			Qup = update_q(demand,I,Si,Fij,Qij),
			{Rup,Uup,Jup} = update_demand_constrained(I,J,Rj,Qup,Uj),
			redistribute_flows(Si,Fij,Rup,Ci,Cij,Uup,Jup,Qup)
	end.

% 4.4 numerical example thesis ruben p 59
test_sub(0)->
	void;
test_sub(N)->
	% turning fractions F
	F = [{1,5,0/1000},{1,6,100/1000},{1,7,300/1000},{1,8,600/1000},{2,5,100/2000},{2,6,0/2000},{2,7,300/2000},{2,8,1600/2000},{3,5,125/1000},{3,6,125/1000},{3,7,0/1000},{3,8,750/1000},{4,5,118/2000},{4,6,941/2000},{4,7,941/2000},{4,8,0/2000}],
	% incoming flow S
	S = [{1,500},{2,2000},{3,800},{4,1700}],
	% outgoing flow R
	R = [{5,1000},{6,2000},{7,1000},{8,2000}],
	% constraints C
	C= [{1,1000},{2,2000},{3,1000},{4,2000}],
	redistribute_flows(S,F,C,R),
	test_sub(N-1).
	
test()->
	timer:tc(node_model,test_sub,[1000000]).
