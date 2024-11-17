#include <vector>
#include <iostream>
#include "cone.cpp"
#include "conePath.cpp"
#pragma once

// Cone BST Class
class ConeBST
{
private:
    struct Node
    {
        Cone cone;
        Node *left;
        Node *right;
        bool finalNode = false;
    };

    Node *root;

    Node *makeEmpty(Node *node)
    {
        if (node == nullptr)
        {
            return nullptr;
        }

        makeEmpty(node->left);
        makeEmpty(node->right);
        delete node;
        return nullptr;
    }

    Node *insert(Node *node, Cone cone)
    {
        if (node == nullptr)
        {
            return new Node{cone, nullptr, nullptr};
        }

        if (cone.getPos().x < node->cone.getPos().x)
        {
            node->left = insert(node->left, cone);
        }
        else
        {
            node->right = insert(node->right, cone);
        }

        return node;
    }

    Node *findMin(Node *node)
    {
        if (node == nullptr)
        {
            return nullptr;
        }

        if (node->left == nullptr)
        {
            return node;
        }

        return findMin(node->left);
    }

    Node *findMax(Node *node)
    {
        if (node != nullptr)
        {
            while (node->right != nullptr)
            {
                node = node->right;
            }
        }

        return node;
    }

    Node *remove(Node *node, Cone cone)
    {
        Node *temp;
        if (node == nullptr)
        {
            return nullptr;
        }
        else if (cone.getPos().x < node->cone.getPos().x)
        {
            node->left = remove(node->left, cone);
        }
        else if (cone.getPos().x > node->cone.getPos().x)
        {
            node->right = remove(node->right, cone);
        }
        else if (node->left && node->right)
        {
            temp = findMin(node->right);
            node->cone = temp->cone;
            node->right = remove(node->right, node->cone);
        }
        else
        {
            temp = node;
            if (node->left == nullptr)
            {
                node = node->right;
            }
            else if (node->right == nullptr)
            {
                node = node->left;
            }
            delete temp;
        }

        return temp;
    }

    void inOrder(Node *node)
    {
        if (node == nullptr)
        {
            return;
        }

        inOrder(node->left);
        std::cout << node->cone.getPos().x << " ";
        inOrder(node->right);
    }

    void toPath(Node *node, std::vector<Cone> &cones)
    {
        if (node == nullptr)
        {
            return;
        }

        toPath(node->left, cones);
        cones.push_back(node->cone);
        toPath(node->right, cones);
    }

    Node *find(Node *node, double x) // NEED TO REWRITE THIS FUNCTION
    {
        if (node == nullptr)
        {
            return nullptr;
        }
        else if (x < node->cone.getPos().x)
        {
            return find(node->left, x);
        }
        else if (x > node->cone.getPos().x)
        {
            return find(node->right, x);
        }
        else
        {
            return node;
        }
    }

public:
    ConeBST()
    {
        root = nullptr;
    }

    ~ConeBST()
    {
        root = makeEmpty(root);
    }

    void insert(Cone cone)
    {
        root = insert(root, cone);
    }

    void remove(Cone cone)
    {
        root = remove(root, cone);
    }

    void display()
    {
        inOrder(root);
        std::cout << std::endl;
    }

    ConePath outputToPath()
    {
        std::vector<Cone> cones;
        toPath(root, cones);
        return ConePath(cones);
    }

    Cone findMin()
    {
        return findMin(root)->cone;
    }

    Cone findMax()
    {
        return findMax(root)->cone;
    }

    Cone find(double x)
    {
        return find(root, x)->cone;
    }
};