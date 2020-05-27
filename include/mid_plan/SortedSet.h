#ifndef SORTEDSET_H
#define SORTEDSET_H
#include <list>
#include <set>

// T must be a pointer, must have member:
// bool inQ;
// std::multiset<nkey>::iterator it;
template<class T>
class SortedSet
{
public:
    typedef std::pair<float, T> nkey;

    struct SortedSetComp
    {
        bool operator() (const nkey& lhs, const nkey& rhs) const
        {
            return lhs.first<rhs.first;
        }
    };

    SortedSet()
    {

    }
    bool insert(T n, float score)
    {
        nkey npair;
        if (!n->inQ) // not in set, just insert
        {
            n->inQ = true;
            npair.first = score;
            npair.second = n;
            n->it = _set.insert(npair);
            return true;
        }
        else
        {
            // in set already, need to remove the old value
            _set.erase(n->it);

            // then insert again
            npair.first = score;
            npair.second = n;
            n->it = _set.insert(npair);
            return false;
        }
    }
    //---
    T pop()
    {
        if (_set.size()>0)
        {
            T n = static_cast<T>(_set.begin()->second);
            _set.erase(_set.begin());
            n->inQ = false;
            return n;
        }
        else
        {
            return nullptr;
        }
    }
    //---
    void clear()
    {
        while(_set.size()>0)
        {
            T n = static_cast<T>(_set.begin()->second);
            _set.erase(_set.begin());
            n->inQ = false;
        }
    }
    //---
    int size()
    {
        return _set.size();
    }
    //---
    T top()
    {
        if (_set.size()>0)
        {
            T n = static_cast<T>(_set.begin()->second);
            return n;
        }
        else
        {
            return nullptr;
        }
    }
    //---
    void remove(T n)
    {
        _set.erase(n->it);
    }

private:
    std::multiset<nkey,SortedSetComp> _set;
};

#endif // SORTEDSET_H
