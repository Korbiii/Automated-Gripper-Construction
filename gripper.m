clf;
clear;
SG_base = SGTrigripper;
SG_grips = SGgriper;
SG_conn = SGconnection1;
SG_conn2 = SGconnection2;
SG_servo = SGServomount;
SG_conn_rod = SGconnRod;


fc = SGTframeChain(1,[1 'F1' 2 'B' 1 'F2' 2 'B' 1 'F3' 2 'B' 2 'F' 5 'B' 3 'F' 6 'B' 4 'F' 7 'B' 5 'F' 8 'B' 6 'F' 9 'B' 7 'F' 10 'B' 1 'F' 11 'B']);

SG = SGTchain({SG_base,SG_grips,SG_grips,SG_grips,SG_conn,SG_conn,SG_conn,SG_conn2,SG_conn2,SG_conn2,SG_servo},[0],'',fc);

SGplot(SG);