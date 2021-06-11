namespace Debugging
{
    public static class Debug
    {
        public static void Log(object message)
        {
#if UNITY_EDITOR
            UnityEngine.Debug.Log(message);
#elif DEBUG
            System.Diagnostics.Debug.WriteLine(message);
#endif
        }
        
        public static void Assert(bool condition, string message)
        {
#if UNITY_EDITOR
            UnityEngine.Debug.Assert(condition, message);
#elif DEBUG
            System.Diagnostics.Debug.Assert(condition, message);
#endif
        }

        public static void Assert(bool condition)
        {
            Assert(condition, null);
        }
    }
}
