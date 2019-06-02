#include <bits/stdc++.h>
using namespace std;
double dist(double x1,double y1,double x2,double y2);
class packet{
    int source;
    int destination;
    double source_x;
    double source_y;
    double destination_x;
    double destination_y;
    double line_dis;
    int last_hop;
    int next_hop;
    int previous_intersection_point;
public:
    packet(int new_source,int new_destination,double new_s_x,double new_s_y,double new_d_x,double new_d_y);
    int read_source();
    int read_destination();
    int read_last_hop();
    int read_next_hop();
    int read_previous_intersection_point();
    double read_source_x();
    double read_source_y();
    double read_destination_x();
    double read_destination_y();
    double read_line_dis();
    void change_last_hop(int prehop);
    void change_next_hop(int newhop);
    void change_previous_intersection_point(int closer);
    void change_line_dis(double dis);
};
packet::packet(int new_source,int new_destination,double new_s_x,double new_s_y,double new_d_x,double new_d_y){
    source=new_source;
    destination=new_destination;
    source_x=new_s_x;
    line_dis=dist(new_s_x,new_s_y,new_d_x,new_d_x);
    source_y=new_s_y;
    destination_x=new_d_x;
    destination_y=new_d_y;
    last_hop=new_source;
    next_hop=new_source;
    previous_intersection_point=new_source;
}
int packet::read_source(){
    return source;
}
int packet::read_destination(){
    return destination;
}
int packet::read_last_hop(){
    return last_hop;
}
int packet::read_next_hop(){
    return next_hop;
}
int packet::read_previous_intersection_point(){
    return previous_intersection_point;
}
double packet::read_source_x(){
    return source_x;
}
double packet::read_source_y(){
    return source_y;
}
double packet::read_destination_x(){
    return destination_x;
}
double packet::read_destination_y(){
    return destination_y;
}
double packet::read_line_dis(){
    return line_dis;
}
void packet::change_last_hop(int prehop){
    last_hop=prehop;
}
void packet::change_next_hop(int newhop){
    next_hop=newhop;
}
void packet::change_previous_intersection_point(int closer){
    previous_intersection_point=closer;
}
void packet::change_line_dis(double dis){
    line_dis=dis;
}
class node{
    int id;
    double x;
    double y;
public:
    friend class packet;
    vector<node> neighbor;
    vector<node> neighbor_planer;
    queue<packet> send_packets;
    node(int newID, double newX, double newY);
    int read_id();
    double read_x();
    double read_y();
    bool checkQueue();
    int getNextHop();
    void send(packet P,vector <node> &R);
};
node::node(int newID, double newX, double newY){
    id=newID;
    x=newX;
    y=newY;
}
int node::read_id(){
    return id;
}
double node::read_x(){
    return x;
}
double node::read_y(){
    return y;
}
bool node::checkQueue(){
    if(send_packets.empty()==false){
        if(send_packets.front().read_next_hop()==id){
            send_packets.front().change_next_hop(getNextHop());
            send_packets.front().change_last_hop(id);
            return true;
        }
        else{
            send_packets.pop();
            return false;
        }
    }
    else
    	return false;
}
int node::getNextHop(){
    double LorR(double x1,double y1,double x2,double y2,double x3,double y3);

    int i,j,s,d,sel_next,flag=1;
    double s_x,s_y,d_x,d_y,v1_x,v1_y,v2_x,v2_y,mid_x,mid_y,inn,angle,min_angle,m1,m2;
    vector <bool> sting;
    for(i=0;i<neighbor_planer.size();i++)
    	sting.push_back(true);

    s=send_packets.front().read_source();
    d=send_packets.front().read_destination();
    s_x=send_packets.front().read_source_x();
    s_y=send_packets.front().read_source_y();
    d_x=send_packets.front().read_destination_x();
    d_y=send_packets.front().read_destination_y();

    if(id==send_packets.front().read_source()){
        v1_x=d_x-s_x;
        v1_y=d_y-s_y;
    }
    else{
        for(i=0;i<neighbor_planer.size();i++){
            if(send_packets.front().read_last_hop()==neighbor_planer[i].read_id())
                break;
        }
        v1_x=neighbor_planer[i].read_x()-x;
        v1_y=neighbor_planer[i].read_y()-y;
    }
    for(j=0;j<neighbor_planer.size();j++)
        sting.push_back(true);
    while(flag){
        min_angle=10*M_PI;
        for(j=0;j<neighbor_planer.size();j++){
            v2_x=neighbor_planer[j].read_x()-x;
            v2_y=neighbor_planer[j].read_y()-y;
            inn=v1_x*v2_x+v1_y*v2_y;
            if(send_packets.front().read_last_hop()==neighbor_planer[j].read_id())
				angle=2*M_PI;
			else
				angle=acos(inn/(sqrt(v1_x*v1_x+v1_y*v1_y)*sqrt(v2_x*v2_x+v2_y*v2_y)));
            if((v1_x*v2_y-v1_y*v2_x)>0)
                angle=2*M_PI-angle;
            //cout << angle << ' ' << neighbor_planer[j].read_id() << endl;
            if(angle<min_angle&&sting[j]==true){
                min_angle=angle;
                sel_next=j;
            }//cout << neighbor_planer[sel_next].read_id()<<endl;
        }
        m1=(d_y-s_y)/(d_x-s_x);
        m2=(neighbor_planer[sel_next].read_y()-y)/(neighbor_planer[sel_next].read_x()-x);
        mid_x=(y-m2*x-(s_y-m1*s_x))/(m1-m2);
        mid_y=m1*mid_x+(s_y-m1*s_x);
        if((fabs(mid_x-s_x)<fabs(d_x-s_x)||fabs(mid_y-s_y)<fabs(d_y-s_y))&&LorR(s_x,s_y,d_x,d_y,x,y)*LorR(s_x,s_y,d_x,d_y,neighbor_planer[sel_next].read_x(),neighbor_planer[sel_next].read_y())<0&&dist(mid_x,mid_y,d_x,d_y)<send_packets.front().read_line_dis()){
            send_packets.front().change_previous_intersection_point(neighbor_planer[sel_next].read_id());
            send_packets.front().change_line_dis(dist(mid_x,mid_y,d_x,d_y));
            if(LorR(s_x,s_y,d_x,d_y,neighbor_planer[sel_next].read_x(),neighbor_planer[sel_next].read_y())<0){
                v1_x=x-neighbor_planer[sel_next].read_x();
                v1_y=y-neighbor_planer[sel_next].read_y();
                sting[sel_next]=false;
            }
            else{
                flag=0;
                break;
            }
        }
        else
            flag=0;
    }

    if(neighbor_planer[sel_next].read_id()==send_packets.front().read_destination())
        return -1;
    else
        return neighbor_planer[sel_next].read_id();
}
void node::send(packet P,vector <node> &R){
    int k;
    for(k=0;k<neighbor_planer.size();k++){
        R[neighbor_planer[k].read_id()].send_packets.push(P);
    }
}
int main(){
    ifstream inStream;
    ofstream outStream;
    inStream.open("request.txt");
    outStream.open("result.txt");
    int i,j,k,id,flag,tempj_id,tempk_id,routers,packets;
    double x,y,middle_x,middle_y,dist_ij;
    vector <node> router;
    inStream >> routers;
    for(i=0;i<routers;i++){
        inStream >> id >> x >> y;
        router.push_back(node(id,x,y));
        for(j=0;j<router.size()-1;j++){
            dist_ij=dist(router[i].read_x(),router[i].read_y(),router[j].read_x(),router[j].read_y());
            if(dist_ij<=1||fabs(dist_ij-1)<=0.000001){
                router[i].neighbor.push_back(router[j]);
                router[j].neighbor.push_back(router[i]);
            }
        }
    }
    for(i=0;i<router.size();i++){
        for(j=0;j<router[i].neighbor.size();j++){
            flag=1;
            tempj_id=router[i].neighbor[j].read_id();
            middle_x=(router[i].read_x()+router[tempj_id].read_x())/2;
            middle_y=(router[i].read_y()+router[tempj_id].read_y())/2;
            dist_ij=dist(middle_x,middle_y,router[i].read_x(),router[i].read_y());
            for(k=0;k<router[i].neighbor.size();k++){
                tempk_id=router[i].neighbor[k].read_id();
                if(j!=k&&dist_ij>=dist(middle_x,middle_y,router[tempk_id].read_x(),router[tempk_id].read_y())){
                    flag=0;
                    break;
                }
            }
            if(flag==1){
                router[i].neighbor_planer.push_back(router[tempj_id]);
            }
        }
    }
    inStream >> packets;
    outStream << packets << endl;
    for(i=0;i<packets;i++){
        inStream >> j >> k;
        router[j].send_packets.push(packet(j,k,router[j].read_x(),router[j].read_y(),router[k].read_x(),router[k].read_y()));
        outStream << router[j].send_packets.front().read_source() << ' ';
        while(1){
        	flag=1;
            for(j=0;j<router.size();j++){
                if(router[j].checkQueue()){
                    if(router[j].send_packets.front().read_next_hop()==-1)
                        outStream << router[j].send_packets.front().read_destination();
                    else
                        outStream << router[j].send_packets.front().read_next_hop() << ' ';
		    		router[j].send(router[j].send_packets.front(),router);
                }
            }

            for(j=0;j<router.size();j++){
                if(router[j].send_packets.empty()==false){
                    flag=0;
                    break;
                }
            }
            if(flag==1)
                break;
        }
        outStream << endl;
    }
    inStream.close();
    outStream.close();
}
double dist(double x1,double y1,double x2,double y2){
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}
double LorR(double x1,double y1,double x2,double y2,double x3,double y3){
    return (x3-x1)/(x2-x1)-(y3-y1)/(y2-y1);
}
