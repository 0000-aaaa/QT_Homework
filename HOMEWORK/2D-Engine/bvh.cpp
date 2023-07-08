
#include "bvh.h"
#include <QDebug>
BVH::BVH(){
    parent = NULL;
    body = NULL;
    children[0] = NULL;
    children[1] = NULL;
}

BVH::BVH(BVH *_parent, BoundingVolume _volume, Polygon *_body):volume(_volume)
{
    parent = _parent;
    body = _body;
    children[0] = NULL;
    children[1] = NULL;
}

BVH::~BVH()
{
    if(parent)
    {
        BVH* sibling;
        if(parent->children[0]==this)sibling = parent->children[1];
        else sibling = parent->children[0];
        parent->volume = sibling->volume;
        parent->body = sibling->body;
        parent->children[0] = sibling->children[0];
        parent->children[1] = sibling->children[1];
        sibling->parent = NULL;
        sibling->body = NULL;
        sibling->children[0] = NULL;
        sibling->children[1] = NULL;
        delete sibling;
        parent->recaculateBoundingVolume();
    }
    if(children[0]){
        children[0]->parent = NULL;
        delete children[0];
    }
    if(children[1]){
        children[1]->parent = NULL;
        delete children[1];
    }
}

bool BVH::isLeaf() const
{
    return (body!=NULL);
}

bool BVH::overlaps(const BVH *other) const
{
    return volume.overlaps(other->volume);
}

unsigned int BVH::getPotentialContactsWith(const BVH *other, PotentialContact *contacts, unsigned int limit) const
{
    if(!overlaps(other)||limit==0)return 0;
    if(isLeaf()&&other->isLeaf())
    {
        contacts->body[0] = body;
        contacts->body[1] = other->body;
        return 1;
    }
    if(other->isLeaf()
        ||(!isLeaf()&&volume.getSize()>=other->volume.getSize()))
    {
        unsigned count = children[0]->getPotentialContactsWith(
            other, contacts, limit
            );
        if(limit > count){
            return count + children[1]->getPotentialContactsWith(
                       other, contacts+count,limit-count
                       );
        }
        else{
            return count;
        }
    }
    else
    {
        unsigned count = getPotentialContactsWith(
            other->children[0], contacts, limit
            );
        if(limit > count){
            return count + getPotentialContactsWith(
                       other->children[1], contacts+count,limit-count
                       );
        }
        else{
            return count;
        }
    }
}

unsigned int BVH::getPotentialContacts(PotentialContact *contacts, unsigned int limit)
{
    if(isLeaf()||limit==0)return 0;
    unsigned int num1 = children[0]->getPotentialContactsWith(children[1],contacts,limit);
    unsigned int num2 = children[0]->getPotentialContacts(contacts+num1,limit-num1);
    unsigned int num3 = children[1]->getPotentialContacts(contacts+num1+num2,limit-num1-num2);
    return num1+num2+num3;
}

void BVH::recaculateBoundingVolume()
{
    if(isLeaf()){
        volume = BoundingVolume(*body);
    }
    else{
        volume.left = children[0]->volume.left<=children[1]->volume.left?
                          children[0]->volume.left:children[1]->volume.left;
        volume.right = children[0]->volume.right>=children[1]->volume.right?
                           children[0]->volume.right:children[1]->volume.right;
        volume.up = children[0]->volume.up>=children[1]->volume.up?
                        children[0]->volume.up:children[1]->volume.up;
        volume.down = children[0]->volume.down<=children[1]->volume.down?
                          children[0]->volume.down:children[1]->volume.down;
    }
}

void BVH::insert(Polygon *newBody, const BoundingVolume &newVolume)
{
    if(isLeaf())
    {
        children[0] = new BVH(this,volume,body);
        children[1] = new BVH(this,newVolume,newBody);
        this->body = NULL;
        recaculateBoundingVolume();
    }
    else
    {
        if(children[0]->volume.getGrowth(newVolume)<
            children[1]->volume.getGrowth(newVolume))
        {
            children[0]->insert(newBody,newVolume);
            recaculateBoundingVolume();
        }
        else
        {
            children[1]->insert(newBody,newVolume);
            recaculateBoundingVolume();
        }
    }
}

void BVH::clear()
{
    if(isLeaf()){
        body = NULL;
        parent = NULL;
    }
    else{
        children[0]->clear();
        children[1]->clear();
        children[0] = NULL;
        children[1] = NULL;
        volume = BoundingVolume(0,0,0,0);
    }
}

BVH* BVH::changeNode()
{
    BVH* ans = NULL;
    if(isLeaf())
    {
        if(volume.left!=body->left||
            volume.right!=body->right||
            volume.up!=body->up||
            volume.down!=body->down)
        {
            ans = this;
        }
    }
    else
    {
        ans = children[0]->changeNode();
        if(!ans)ans = children[1]->changeNode();
    }
    return ans;
}

void BVH::moveNode()
{
    BVH* temp=changeNode();
    Polygon* tempBody = NULL;
    int i = 0;
    while(temp){
        tempBody = temp->body;
        delete temp;
        insert(tempBody,BoundingVolume((*tempBody)));
        temp = changeNode();
        qDebug()<<++i;
    }
}

void BVH::operator=(const BVH &copy)
{
    body = copy.body;
    parent = copy.parent;
    volume = copy.volume;
    children[0] = copy.children[0];
    children[1] = copy.children[1];
}

