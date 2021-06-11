using System;

// Barebones performant struct version of List<T>
// Remember this is a struct, so pass by reference has to be explicit with ref keyword
public struct DynamicArray<T>
{
    public T[] arr;
    public int Count { get; private set; }

    public T this[int index]
    {
        get => arr[index];
        set => arr[index] = value;
    }

    public DynamicArray(int capacity)
    {
        arr = new T[capacity];
        Count = 0;
    }

    public void Add(T item)
    {
        if (arr.Length == Count)
            Array.Resize(ref arr, arr.Length == 0 ? 1 : (arr.Length * 2));
        arr[Count++] = item;
    }

    public void Swap(int index1, int index2)
    {
        var temp = arr[index1];
        arr[index1] = arr[index2];
        arr[index2] = temp;
    }

    public void SwapRemoveAt(int index)
    {
        Swap(index, Count - 1);
        RemoveLast();
    }

    public void RemoveLast()
    {
        Count--;
        Debugging.Debug.Assert(Count >= 0);
    }

    public void Clear()
    {
        Count = 0;
    }

    public void ClearOrInit(int capacity)
    {
        if (arr == null)
            arr = new T[capacity];
        else
            Clear();
    }

    public T Last()
    {
        return arr[Count - 1];
    }

    public bool Extend(int num)
    {
        Count += num;
        if (Count > arr.Length)
        {
            Count -= num;
            return false;
        }
        return true;
    }

    public void Resize(int capacity)
    {
        if (arr.Length < capacity)
            Array.Resize(ref arr, capacity);
    }

    public void ResizeAndExtendTo(int capacity)
    {
        Resize(capacity);
        Count = capacity;
    }
}
