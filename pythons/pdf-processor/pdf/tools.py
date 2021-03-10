def get_descendants_recursively(clazz, descendants=None):
    if descendants is None:
        descendants = []
    for subclass in clazz.__subclasses__():
        get_descendants_recursively(subclass, descendants)
        descendants.append(subclass)
    return descendants


if __name__ == '__main__':
    class A:
        pass


    class B(A):
        pass


    class C(B):
        pass


    class D(B):
        pass


    print(get_descendants_recursively(A))
