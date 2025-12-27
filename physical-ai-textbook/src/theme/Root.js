import React from 'react';
import HuggingFaceChat from '@site/src/components/HuggingFaceChat';

export default function Root({children}) {
  return (
    <>
      {children}
      <HuggingFaceChat />
    </>
  );
}