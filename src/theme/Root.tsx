import React from 'react';
import RAGChatbot from '@site/src/components/RAGChatbot';

// Default implementation, that you can customize
export default function Root({children}: {children: React.ReactNode}): React.ReactElement {
  return (
    <>
      {children}
      <RAGChatbot />
    </>
  );
}
