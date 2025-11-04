using System;
using UnityEngine;
using UnityEngine.UIElements;

namespace OnlineFPS
{
    public static class UIUtilities
    {
        public static void SetDisplay(this VisualElement element, bool enabled)
        {
            element.style.display = enabled ? DisplayStyle.Flex : DisplayStyle.None;
        }
    }
}